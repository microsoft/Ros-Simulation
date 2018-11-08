#!/bin/bash

# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

RESOURCE_GROUP=""
LOCATION=""
KUBERNETES_FILE="./kubernetes.json"
CLUSTER_NAME=""
SUBSCRIPTION_ID=""

ORCHESTRATOR_VERSION="1.10.5"

# Master and Agent pool profiles for running ~100 robots
MASTER_VM_SIZE="Standard_D32s_v3"
SIMULATOR_VM_SIZE="Standard_M128s"
ROBOT_VM_SIZE="Standard_D16s_v3"
ROBOT_VM_COUNT=15

SSH_KEY_DATA=""
SERVICE_PRINCIPAL_CLIENT_ID=""

#---------------------------------------------------------------------------------------
# You could use secret in plain text or use Azure Key Vault service to store your secret
# Secret in plain text
SERVICE_PRINCIPAL_SECRET=""

# Or store the information in Azure Key Vault service
KEYVAULT_NAME=""
SERVICE_PRINCIPAL_SECRET_NAME=""
#---------------------------------------------------------------------------------------

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