#!/bin/bash

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

RESOURCE_GROUP="bmw-xiou-acs"
LOCATION="east us"
KUBERNETES_FILE="./kubernetes.json"
CLUSTER_NAME="bmw-xiou-acs"
SUBSCRIPTION_ID="b2dffb01-098e-407e-b075-1a7c1bf68d2e"

ORCHESTRATOR_VERSION="1.10.5"

# Master and Agent pool profiles for running ~100 robots
MASTER_VM_SIZE="Standard_D32s_v3"
SIMULATOR_VM_SIZE="Standard_M128s"
ROBOT_VM_SIZE="Standard_D16s_v3"
ROBOT_VM_COUNT=15

#SSH_KEY_DATA="ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAACAQDl2nu3OLzQibeH5cafIIiL93FrUfln+/uslIeXCU4lyyItUfT+l/s3JbnT0yQLEe24C2v9O+6TE1zsd2wh/73v3NMVkE+BtgtYEqJqxGqyp327CkqSvjdl9G9r2mhUyKZgMYpbMfe+lv81m/OLgdROciA+LfbVlO1v7EbcYx/qytOHtSSdgeVvfxPLJ7qGjnohpTaBG5/SvcwRJjWVWDr0pP+ANzV9vyH7Xk9fyfJruV7PeRa4i4vBjGcGS80eOUegd49tnvBg53QHRqkCKUez/qbD0BiIZfnduhopBBhoXtg1y34v/rZ6LRzYaqc2pE3Y2OT3yGfsx4INTvNYbXu5PLs6LIyTJhs7TtnSdYl3amm1zIxMq4p9+O9Fy/ZtOmz/Bpbs4Fx4Dy6Xv9WsM97M/P/7GSbw1HnKbb4K4GifwVYyltg8xkAypqzHwzwDUfZo3bBoVlLSJsbpqOhmJ8dTK8BsLOVOoEpbeVfPT5x9I8SxKi5vJWOYnbEw0/PbqCYkZYyn0lMzly1UumgpuDIUhnXbIKPMCF13VFGi7lT8Nr7BDGaQ2tPD5+MctxaNwXt3ZaSPcIBlB/fHcYAwL2ScNrx6qNBl0LOPHz9/e5MlwREDmHI1dXXUUXFzgwXF8eaIKjkU8gesPVe18wEzse7Ui0DB1EyO0GyUIrKqmUbtww== azureuser"
SSH_KEY_DATA="ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQDGfS099txHdrkRb4yFKAHszRU1Saw679rpJiOI6JwS+mWSFis8B6ewmoFEbVYWD8ZHjdDLTo50/HuXkK3SEScboyDfUA9ySIkhBorUg+gaDsKsEEofFTYhCjXNyPm3bvu2gDLbhd25M6WMOtKdR5cDUItUOy0q/Dx43a1kIrdsrqdoBzf3aD1gx03+AIrznNx7wtK8Mq2QHnx0jOw/g6/GIpEEkpqQUgRMFTss/75cXU7pnYsaznJzZlypGBNNOU1UgK9ad+gGAePKkwoPnx9iIJfjwsF3DzkXlcAAxwOAwF+B32Cpyy/CmamnnAtW+9f23zHBspC+3wcxDKFnt6DV xiou@XinyisWrkLaptop.guest.corp.microsoft.com"
SERVICE_PRINCIPAL_CLIENT_ID="e21edeee-73f3-45f9-930c-dfbc8b45ab68"

#---------------------------------------------------------------------------------------
# You could use secret in plain text or use Azure Key Vault service to store your secret
# Secret in plain text
SERVICE_PRINCIPAL_SECRET="AzZgv1w8+298mgEN8wExXUeeurO9oumus+lwCsyiHCM="

# Or store the information in Azure Key Vault service
KEYVAULT_NAME=""
SERVICE_PRINCIPAL_SECRET_NAME=""
#---------------------------------------------------------------------------------------

set -euo pipefail

echo "${SSH_KEY_DATA}"

# Set subscription 
az account set --subscription $SUBSCRIPTION_ID

# Create resource group
az group create --name $RESOURCE_GROUP --location "${LOCATION}"

if [ -z "$KEYVAULT_NAME" ] && ! [ -z "$SERVICE_PRINCIPAL_SECRET" ]; then
    echo "No key vault info is given, using secret directly."

    # Generate folder from json file
    acs-engine generate $KUBERNETES_FILE \
        --set orchestratorProfile.orchestratorVersion=$ORCHESTRATOR_VERSION \
        --set masterProfile.dnsPrefix=$CLUSTER_NAME \
        --set masterProfile.vmSize=$MASTER_VM_SIZE \
        --set agentPoolProfiles[0].vmSize=$SIMULATOR_VM_SIZE \
        --set agentPoolProfiles[1].vmSize=$ROBOT_VM_SIZE \
        --set agentPoolProfiles[1].count=$ROBOT_VM_COUNT \
        --set linuxProfile.ssh.publicKeys[0].keyData="${SSH_KEY_DATA}" \
        --set servicePrincipalProfile.clientId=$SERVICE_PRINCIPAL_CLIENT_ID \
        --set servicePrincipalProfile.secret=$SERVICE_PRINCIPAL_SECRET
        
elif ! [ -z "$KEYVAULT_NAME" ]; then
    echo "Key vault is given, then use key vault to get secret."

    vault_ID="/subscriptions/$SUBSCRIPTION_ID/resourceGroups/$RESOURCE_GROUP/providers/Microsoft.KeyVault/vaults/$KEYVAULT_NAME"
    
    acs-engine generate $KUBERNETES_FILE \
        --set orchestratorProfile.orchestratorVersion=$ORCHESTRATOR_VERSION \
        --set masterProfile.dnsPrefix=$CLUSTER_NAME \
        --set masterProfile.vmSize=$MASTER_VM_SIZE \
        --set agentPoolProfiles[0].vmSize=$SIMULATOR_VM_SIZE \
        --set agentPoolProfiles[1].count=$ROBOT_VM_COUNT \
        --set agentPoolProfiles[1].vmSize=$ROBOT_VM_SIZE \
        --set linuxProfile.ssh.publicKeys[0].keyData="${SSH_KEY_DATA}" \
        --set servicePrincipalProfile.clientId=$SERVICE_PRINCIPAL_CLIENT_ID \
        --set servicePrincipalProfile.keyvaultSecretRef.vaultID=$vault_ID \
        --set servicePrincipalProfile.keyvaultSecretRef.SecretName=$SERVICE_PRINCIPAL_SECRET_NAME   
else
    echo "Error, you should either set SERVICE_PRINCIPAL_SECRET or (KEYVAULT_NAME and SERVICE_PRINCIPAL_SECRET_NAME)."

fi

# create deployment from generated files
az group deployment create --resource-group $RESOURCE_GROUP --template-file _output/${CLUSTER_NAME}/azuredeploy.json --parameters _output/${CLUSTER_NAME}/azuredeploy.parameters.json