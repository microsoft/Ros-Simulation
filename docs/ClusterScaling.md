# Cluster Scaling

If you are seeing a low [Real Time Factor (RTF)](./ValidateAndTroubleshoot.md#Real-Time-Factor-(RTF)) of less than .9 (a RTF of .9-1.1 is desireable), you may need to increase the size of the simulator box. In this case, you will need to redeploy your cluster. These scaling steps are for increasing the number of robot nodes, not for increasing the size of the vms.

Keep in mind that the simulation only works with one simulation pod, so there is no reason to scale out the simulation agent pool, except maybe for fail-over (if the actual vm has issues, K8s will automatically choose the other box if it is also tainted).

Each robot requires a certain allocation of memory and cpu to run (declared at bottom of the [values.yaml](../helm/ros-simulation/values.yaml) file). If you try to deploy more robots than there are resources, you will see that some of your pods are in "Pending" status. In this case, calculate the number of cores needed for your robots, and increase the number of worker nodes accordingly.

You may also want to scale down in order to reduce cost when the cluster is not being heavily used.

In order to change the number of nodes in your cluster, there are two methods depending on what type of cluster you have: ACS-engine or AKS.

## ACS-engine Cluster

To scale your ACS-engine cluster, follow the docs [here](https://github.com/Azure/acs-engine/blob/master/docs/kubernetes/scale.md).

Make sure to scale the "robotpool" node-pool.

## AKS Cluster

In order to scale your AKS cluster, follow the docs [here](https://docs.microsoft.com/en-us/azure/aks/scale-cluster).

## Validation

After successful scaling, you should be able to run:

```console
kubectl get nodes
```

If you see the number of nodes that you requested in "Ready" status, then your cluster scaling was successful.
