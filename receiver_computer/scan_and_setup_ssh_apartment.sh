#!/bin/bash

# Step 1: Get local IP and subnet
LOCAL_IP=$(ifconfig | grep 'inet ' | grep -Eo '10\.[0-9]+\.[0-9]+\.[0-9]+' | head -n 1)
if [ -z "$LOCAL_IP" ]; then
    echo "Could not determine local IP. Make sure you're connected to the network."
    exit 1
fi

SUBNET=$(echo $LOCAL_IP | cut -d'.' -f1-3).0/24
echo "Scanning network $SUBNET for active hosts..."

# Step 2: Run nmap to find active hosts on this subnet
ACTIVE_IPS=$(nmap -sn $SUBNET | grep 'Nmap scan report for' | awk '{print $5}')

# Step 3: Define SSH user
SSH_USER="admin"

# Step 4: Loop over each IP to copy the SSH key and connect
for ip in $ACTIVE_IPS; do
    echo "Trying $ip..."

    # Check if the IP allows SSH
    ssh -o ConnectTimeout=3 $SSH_USER@$ip exit
    if [ $? -ne 0 ]; then
        echo "$ip does not accept SSH connections. Skipping."
        continue
    fi

    # Copy SSH key to the remote host (only needs to be done once per host)
    ssh-copy-id -o StrictHostKeyChecking=no $SSH_USER@$ip

    # Attempt a direct SSH session
    echo "Successfully connected to $ip. Opening an interactive session..."
    ssh -o StrictHostKeyChecking=no $SSH_USER@$ip
    break
done
