# Azure Container Service Engine for Kubernetes Deployment

## Why ACS Engine is Needed

The [Azure Container Service Engine](https://github.com/Azure/acs-engine) (ACS Engine) generates ARM (Azure Resource Manager) templates for Docker enabled clusters on Microsoft Azure with your choice of DC/OS, Kubernetes (k8s), OpenShift, Swarm Mode, or Swarm orchestrators. The input to the tool is a cluster definition. The cluster definition (or apimodel) is very similar to (in many cases the same as) the ARM template syntax used to deploy a Microsoft Azure Container Service cluster.

In this project, we have one *simulator* and multiple *robots*. All the *robots* need to communicate with the *simulator*, and the *simulator* needs to do some computation and send the results back to the *robots*. Therefore, we face two main issues related to the *simulator*:

- High network bandwidth usage
- High computation cost

To solve these issues, we utilize ACS Engine since it allows us to leverage the accelerated networking and allocate a large VM box for the simulator. At the time of writing, [Azure Kubernetes Services](https://docs.microsoft.com/en-us/azure/aks/) (AKS) does not yet support accelerated networking or heterogeneous clusters. If a large number of robots is not needed, AKS may be the simpler choice, please refer to [Creating the Cluster with AKS](ClusterCreation_AKS.md) for deployment details.

## Create ACS Engine with Service Principal

### Install pre-requisites

All the commands in this guide require [Azure CLI](https://docs.microsoft.com/en-us/cli/azure/install-azure-cli?view=azure-cli-latest), [kubectl](https://kubernetes.io/docs/tasks/tools/install-kubectl/), [acs-engine](https://github.com/Azure/acs-engine/blob/master/docs/acsengine.md#install-acs-engine) and [ssh key](https://github.com/Azure/acs-engine/blob/master/docs/ssh.md#ssh-key-generation)

### Step 1 Create a new service principal

If you do not have a service principal that has a role as "Contributor", please follow [this](https://github.com/Azure/acs-engine/blob/master/docs/serviceprincipal.md) to create one.

### Step 2 Set the deployment variables

It is recommended to first create a new resource group for the ACS Engine cluster because it generates resources that may be hard to differentiate from other resources already in an existing group. If the resource group does not exist, then the script will automatically create one for you.

To create a resource group, generate the files for the cluster deployment, and deploy the cluster, run [deploymentScript.sh](../acs-engine/deploymentScript.sh) with the following parameters:

| **Field**                     | **Description**                                                                                                                             |
|-------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| RESOURCE_GROUP                | *Azure resource group*  name                                                                                                                |
| LOCATION                      | Region where the cluster will be deployed                                                                                                   |
| KUBERNETES_FILE               | Path to the kubernetes cluster template file                                                                                                |
| CLUSTER_NAME                  | Name of the cluster (must be unique)                                                                                                        |
| SUBSCRIPTION_ID               | Azure subscription ID                                                                                                                       |
| ORCHESTRATOR_VERSION          | *Kubernetes* version (must be >= 1.10.5)                                                                                                    |
| MASTER_VM_SIZE_               | VM size for the Kubernetes master which needs a proper VM box                                                                               |
| SIMULATOR_VM_SIZE             | VM size for the *simulator*  which requires a large VM box since the simulator needs more resources for heavy computation                   |
| ROBOT_VM_SIZE                 | VM boxes for Robot pool. You could choose what ever VM fits your budget as long as they provide enough computation resources for the robots |
| ROBOT_VM_COUNT                | Number of VMs of size above dedicated to the Robots                                                                                         |
| SSH_KEY_DATA                  | Public key of your ssh key which is used to access the k8s cluster                                                                          |
| SERVICE_PRINCIPAL_CLIENT_ID   | *Client ID* generated in Step 1                                                                                                             |
| SERVICE_PRINCIPAL_SECRET      | Plain text of the secret (see below)                                                                                                        |
| KEYVAULT_NAME                 | *Azure Key Vault* name (see below)                                                                                                          |
| SERVICE_PRINCIPAL_SECRET_NAME | Secret name in *Azure Key Vault* (see below)                                                                                                |

There are two approaches to get the corresponding secret of the service principal as follows:

- You could directly set the secret in plain text. Please leave this information blank if using *Azure Key Vault* option.
  - __SERVICE_PRINCIPAL_SECRET__ is the plain text of the secret

- You could store the information in [Azure Key Vault service](https://docs.microsoft.com/en-us/azure/key-vault/quick-create-portal) and use ```az keyvault update -g $RESOURCE_GROUP -n $KeyVault_NAME --enabled-for-template-deployment``` to allow the template deployment to access the secret. Please leave this information blank if using plain text option.

  - __KEYVAULT_NAME__ is the *Azure Key Vault* name
  - __SERVICE_PRINCIPAL_SECRET_NAME__ is the secret name you used to store the service principal secret in *Azure Key Vault*

![CICD Subscription](../images/cicd_setScript.png =600x)

### Step 3 Review VM sizes and the number of nodes

#### WARNING

Please note that the default configuration of the simulator is M series simulator box (Standard_M128s), and 5 robot pool machines (Standard_D2_v2) are created.

### Step 4 run the script

It will take a bit to deploy depending on the machine sizes and counts.

## Set Kubernetes Configuration File for Cluster

The generated kubernetes config file is `acs-engine/_output/<cluster name>/kubeconfig/kubeconfig.<region>.json` under the project folder.

To use kubernetes commands locally,

- You can use the following command line to merge with your previous config file, and you only need to do this when a new cluster is created and is not yet in the config.  
    > ```KUBECONFIG=~/.kube/config:~/{generated config file} kubectl config view --flatten > ~/.kube/tmp && mv  ~/.kube/tmp ~/.kube/config```

- You can switch between the cluster you have using the following command line:
    > ```kubectl config use-context {cluster name}```

## Validation

- Get your k8s cluster information. The urls are formatted: "<https://{your_cluster_name}.{your_location}...",> please check that your cluster name and the location are correct.

    > ```kubectl cluster-info```

    The output will look like the following:

![Cluster Info Output](../images/clusterinfo.png =1200x)

- Get the k8s node information. You will see __one__ master node, __one__ simulator node, and __N__ number of robotpool nodes.
    > ```kubectl get nodes```

    The output will look like the following:

| NAME                                  | STATUS        | ROLES     | AGE       | VERSION    |
| --------------------------------------|:--------------|:----------|:----------|:-----------|
| k8s-master-16651204-0                 | Ready         | master    | 3h        | v1.10.5    |
| k8s-robotpool-16651204-vmss000000    | Ready         | agent     | 3h        | v1.10.5    |
| k8s-robotpool-16651204-vmss000001    | Ready         | agent     | 3h        | v1.10.5    |
| k8s-simulator-16651204-vmss000000     | Ready         | agent     | 3h        | v1.10.5    |
