#!/usr/bin/env python3
"""
Thread-safe buffer implementation for sensor data exchange.

This module provides thread-safe buffer implementations for exchanging data
between sensor reading threads and the control loop. It is designed to
prevent data loss while maintaining real-time performance.
"""

import threading
import queue
import time
import logging
from collections import deque
from typing import Any, Dict, List, Optional, Tuple, TypeVar, Generic

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("ThreadSafeBuffer")

T = TypeVar('T')  # Generic type for buffer contents

class ThreadSafeBuffer(Generic[T]):
    """
    Thread-safe buffer for data exchange between threads.
    
    This class implements a fixed-size buffer with configurable overflow behavior.
    It is thread-safe and designed for producer-consumer patterns where multiple
    threads may be producing or consuming data.
    """
    
    class OverflowBehavior:
        """Strategies for handling buffer overflow"""
        DROP_OLDEST = "drop_oldest"
        DROP_NEWEST = "drop_newest"
        BLOCK = "block"
    
    def __init__(
        self, 
        maxsize: int = 100, 
        overflow_behavior: str = OverflowBehavior.DROP_OLDEST,
        name: str = "buffer"
    ):
        """
        Initialize the buffer.
        
        Args:
            maxsize: Maximum buffer size before overflow handling is triggered
            overflow_behavior: How to handle overflow (drop_oldest, drop_newest, block)
            name: Name for logging and debugging
        """
        self.name = name
        self.maxsize = maxsize
        self.overflow_behavior = overflow_behavior
        
        # Use a queue for BLOCK behavior
        if overflow_behavior == self.OverflowBehavior.BLOCK:
            self.buffer = queue.Queue(maxsize=maxsize)
        # Use a deque for DROP behaviors (thread-safe for append/pop, not for random access)
        else:
            self.buffer = deque(maxlen=maxsize if overflow_behavior == self.OverflowBehavior.DROP_OLDEST else None)
        
        # Mutex lock for operations that are not inherently thread-safe
        self.lock = threading.Lock()
        
        # Statistics
        self.dropped_count = 0
        self.total_added = 0
        self.total_retrieved = 0
        self.high_water_mark = 0
        self.last_overflow_time = 0
        
    def add(self, item: T, timeout: Optional[float] = None) -> bool:
        """
        Add an item to the buffer.
        
        Args:
            item: The item to add
            timeout: Optional timeout for blocking behavior (seconds)
            
        Returns:
            True if added successfully, False if dropped
        """
        with self.lock:
            self.total_added += 1
        
        try:
            # Handle different overflow behaviors
            if self.overflow_behavior == self.OverflowBehavior.BLOCK:
                # For blocking behavior, use the queue's blocking put
                try:
                    self.buffer.put(item, block=timeout is not None, timeout=timeout)
                    self._update_stats()
                    return True
                except queue.Full:
                    with self.lock:
                        self.dropped_count += 1
                        current_time = time.time()
                        if current_time - self.last_overflow_time > 1.0:
                            logger.warning(f"{self.name}: Buffer full, dropped item (blocking)")
                            self.last_overflow_time = current_time
                    return False
                    
            elif self.overflow_behavior == self.OverflowBehavior.DROP_OLDEST:
                # For drop oldest, use the deque's maxlen feature which automatically
                # drops the oldest item when full
                with self.lock:
                    if len(self.buffer) >= self.maxsize:
                        self.dropped_count += 1
                        current_time = time.time()
                        if current_time - self.last_overflow_time > 1.0:
                            logger.warning(f"{self.name}: Buffer full, dropped oldest item")
                            self.last_overflow_time = current_time
                    self.buffer.append(item)
                    self._update_stats()
                    return True
                    
            elif self.overflow_behavior == self.OverflowBehavior.DROP_NEWEST:
                # For drop newest, check if full and drop the new item if it is
                with self.lock:
                    if len(self.buffer) >= self.maxsize:
                        self.dropped_count += 1
                        current_time = time.time()
                        if current_time - self.last_overflow_time > 1.0:
                            logger.warning(f"{self.name}: Buffer full, dropped newest item")
                            self.last_overflow_time = current_time
                        return False
                    else:
                        self.buffer.append(item)
                        self._update_stats()
                        return True
            else:
                logger.error(f"Unknown overflow behavior: {self.overflow_behavior}")
                return False
                
        except Exception as e:
            logger.error(f"Error adding to buffer: {e}")
            with self.lock:
                self.dropped_count += 1
            return False
    
    def get(self, block: bool = True, timeout: Optional[float] = None) -> Optional[T]:
        """
        Get an item from the buffer.
        
        Args:
            block: Whether to block if buffer is empty
            timeout: Timeout for blocking get (seconds)
            
        Returns:
            The item or None if buffer is empty or timeout occurs
        """
        try:
            # Handle different buffer types
            if self.overflow_behavior == self.OverflowBehavior.BLOCK:
                # For blocking behavior, use the queue's blocking get
                try:
                    item = self.buffer.get(block=block, timeout=timeout)
                    with self.lock:
                        self.total_retrieved += 1
                    return item
                except queue.Empty:
                    return None
            else:
                # For non-blocking behaviors, use the deque
                with self.lock:
                    if not self.buffer:
                        return None
                    item = self.buffer.popleft()
                    self.total_retrieved += 1
                    return item
                    
        except Exception as e:
            logger.error(f"Error getting from buffer: {e}")
            return None
    
    def peek(self) -> Optional[T]:
        """
        Peek at the next item without removing it.
        
        Returns:
            The next item or None if buffer is empty
        """
        with self.lock:
            if self.overflow_behavior == self.OverflowBehavior.BLOCK:
                # Using a queue, which doesn't support peek directly
                # Try to get and put back, but only if not empty
                if self.buffer.qsize() == 0:
                    return None
                
                try:
                    # Non-blocking get
                    item = self.buffer.get(block=False)
                    # Put it back at the front (by creating a temporary new queue)
                    temp_queue = queue.Queue()
                    temp_queue.put(item)
                    # Transfer the remaining items
                    while not self.buffer.empty():
                        temp_queue.put(self.buffer.get())
                    # Transfer back to the original queue
                    while not temp_queue.empty():
                        self.buffer.put(temp_queue.get())
                    return item
                except queue.Empty:
                    return None
            else:
                # Using a deque, which supports peek
                if not self.buffer:
                    return None
                return self.buffer[0]
    
    def clear(self) -> None:
        """Clear the buffer"""
        with self.lock:
            if self.overflow_behavior == self.OverflowBehavior.BLOCK:
                # Clear the queue
                while not self.buffer.empty():
                    try:
                        self.buffer.get(block=False)
                    except queue.Empty:
                        break
            else:
                # Clear the deque
                self.buffer.clear()
    
    def size(self) -> int:
        """Get the current buffer size"""
        with self.lock:
            if self.overflow_behavior == self.OverflowBehavior.BLOCK:
                return self.buffer.qsize()
            else:
                return len(self.buffer)
    
    def is_empty(self) -> bool:
        """Check if the buffer is empty"""
        with self.lock:
            if self.overflow_behavior == self.OverflowBehavior.BLOCK:
                return self.buffer.empty()
            else:
                return len(self.buffer) == 0
    
    def is_full(self) -> bool:
        """Check if the buffer is full"""
        with self.lock:
            if self.overflow_behavior == self.OverflowBehavior.BLOCK:
                return self.buffer.full()
            else:
                return len(self.buffer) >= self.maxsize
    
    def get_stats(self) -> Dict[str, Any]:
        """Get buffer statistics"""
        with self.lock:
            if self.overflow_behavior == self.OverflowBehavior.BLOCK:
                current_size = self.buffer.qsize()
            else:
                current_size = len(self.buffer)
                
            return {
                "name": self.name,
                "maxsize": self.maxsize,
                "current_size": current_size,
                "dropped_count": self.dropped_count,
                "total_added": self.total_added,
                "total_retrieved": self.total_retrieved,
                "high_water_mark": self.high_water_mark,
                "overflow_behavior": self.overflow_behavior
            }
    
    def _update_stats(self) -> None:
        """Update buffer statistics"""
        # Record high water mark
        current_size = self.size()
        if current_size > self.high_water_mark:
            self.high_water_mark = current_size

class TimestampedBuffer(ThreadSafeBuffer):
    """
    Thread-safe buffer with automatic timestamping.
    
    This subclass adds timestamps to items and provides methods to retrieve
    items based on timestamps.
    """
    
    def __init__(
        self, 
        maxsize: int = 100, 
        overflow_behavior: str = ThreadSafeBuffer.OverflowBehavior.DROP_OLDEST,
        name: str = "timestamped_buffer"
    ):
        """
        Initialize the timestamped buffer.
        
        Args:
            maxsize: Maximum buffer size before overflow handling is triggered
            overflow_behavior: How to handle overflow (drop_oldest, drop_newest, block)
            name: Name for logging and debugging
        """
        super().__init__(maxsize, overflow_behavior, name)
        self.latest_timestamp = 0.0
    
    def add(self, item: T, timestamp: Optional[float] = None, timeout: Optional[float] = None) -> bool:
        """
        Add a timestamped item to the buffer.
        
        Args:
            item: The item to add
            timestamp: Optional timestamp (defaults to current time)
            timeout: Optional timeout for blocking behavior
            
        Returns:
            True if added successfully, False if dropped
        """
        if timestamp is None:
            timestamp = time.time()
            
        # Update latest timestamp
        if timestamp > self.latest_timestamp:
            with self.lock:
                self.latest_timestamp = timestamp
                
        # Add the timestamped item
        timestamped_item = (timestamp, item)
        return super().add(timestamped_item, timeout)
    
    def get(self, block: bool = True, timeout: Optional[float] = None) -> Optional[Tuple[float, T]]:
        """
        Get a timestamped item from the buffer.
        
        Args:
            block: Whether to block if buffer is empty
            timeout: Timeout for blocking get
            
        Returns:
            Tuple of (timestamp, item) or None if buffer is empty
        """
        return super().get(block, timeout)
    
    def get_latest(self) -> Optional[Tuple[float, T]]:
        """
        Get the latest item (by timestamp) without removing it.
        
        Returns:
            The latest (timestamp, item) or None if buffer is empty
        """
        with self.lock:
            if self.overflow_behavior == self.OverflowBehavior.BLOCK:
                # For queue, we need to search through all items
                if self.buffer.empty():
                    return None
                    
                # Get all items
                items = []
                while not self.buffer.empty():
                    items.append(self.buffer.get())
                    
                # Find the latest item
                latest_item = max(items, key=lambda x: x[0])
                
                # Put items back
                for item in items:
                    self.buffer.put(item)
                    
                return latest_item
            else:
                # For deque, we can scan more efficiently
                if not self.buffer:
                    return None
                    
                return max(self.buffer, key=lambda x: x[0])
    
    def get_items_since(self, timestamp: float) -> List[Tuple[float, T]]:
        """
        Get all items with timestamps newer than the specified time.
        
        Args:
            timestamp: The reference timestamp
            
        Returns:
            List of (timestamp, item) pairs newer than the reference time
        """
        with self.lock:
            if self.overflow_behavior == self.OverflowBehavior.BLOCK:
                # For queue, we need to get all items and filter
                if self.buffer.empty():
                    return []
                    
                # Get all items
                items = []
                while not self.buffer.empty():
                    items.append(self.buffer.get())
                    
                # Filter items
                newer_items = [item for item in items if item[0] > timestamp]
                
                # Put items back
                for item in items:
                    self.buffer.put(item)
                    
                return newer_items
            else:
                # For deque, we can filter in place
                return [item for item in self.buffer if item[0] > timestamp]
                
    def get_items_between(self, start_time: float, end_time: float) -> List[Tuple[float, T]]:
        """
        Get all items with timestamps between the specified times.
        
        Args:
            start_time: The start timestamp (inclusive)
            end_time: The end timestamp (inclusive)
            
        Returns:
            List of (timestamp, item) pairs within the time range
        """
        with self.lock:
            if self.overflow_behavior == self.OverflowBehavior.BLOCK:
                # For queue, we need to get all items and filter
                if self.buffer.empty():
                    return []
                    
                # Get all items
                items = []
                while not self.buffer.empty():
                    items.append(self.buffer.get())
                    
                # Filter items
                filtered_items = [item for item in items if start_time <= item[0] <= end_time]
                
                # Put items back
                for item in items:
                    self.buffer.put(item)
                    
                return filtered_items
            else:
                # For deque, we can filter in place
                return [item for item in self.buffer if start_time <= item[0] <= end_time]

# Example usage
if __name__ == "__main__":
    # Create a thread-safe buffer
    buffer = ThreadSafeBuffer[int](maxsize=5, name="test_buffer")
    
    # Add some items
    for i in range(10):
        success = buffer.add(i)
        print(f"Added {i}: {success}")
        
    # Get stats
    print(f"\nStats: {buffer.get_stats()}")
    
    # Get all items
    print("\nGetting items:")
    while not buffer.is_empty():
        item = buffer.get()
        print(f"Got {item}")
        
    # Create a timestamped buffer
    ts_buffer = TimestampedBuffer[str](maxsize=5, name="timestamped_buffer")
    
    # Add items with timestamps
    ts_buffer.add("Item 1", timestamp=1000.0)
    ts_buffer.add("Item 2", timestamp=2000.0)
    ts_buffer.add("Item 3", timestamp=1500.0)
    
    # Get latest item
    latest = ts_buffer.get_latest()
    print(f"\nLatest item: {latest}")
    
    # Get items since timestamp
    items = ts_buffer.get_items_since(1200.0)
    print(f"\nItems since 1200.0: {items}")
    
    print(f"\nTimestamped buffer stats: {ts_buffer.get_stats()}")