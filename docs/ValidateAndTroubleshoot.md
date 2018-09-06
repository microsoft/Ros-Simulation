# Validate and Troubleshoot

## Validate Simulation Health

After the project is successfully up and running you may run into various problems and errors. When problems arise start by verifying the status of your nodes and the pods running on them.

To display the status of nodes:
`kubectl get nodes`

To display the status of pods:
`kubectl get pods`

There should be 1 simulator pod, and N number of robot pods (based on values.yaml).

## Real Time Factor (RTF)

Real time factor (RTF) is a representation of how closely the simulation is running at real time speed (where 1 second of simulation = 1 second of real time elapsed).

A RTF in the range of ~0.9 to ~1.1 is desirable.

If the RTF is significantly below 1, then the simulator is running slower than real time. This is because the simulator does not have enough cores or threads to process the work efficiently. Try:

- Increasing the number of cores for the simulator box
- Increasing the number of threads to match the number of cores
- Lowering the number of robots

If the RTF is significantly above 1, then your simulator is running faster than real time. Depending on how well your robot code can adjust to faster than real time, this may or may not be desirable. Try increasing the callbackWaitTimeout closer to 0.1, which will ensure that the simulation will wait at most 1/10 of a second before continuing on the next cycle.

## View Container Logs

Check the logs of the pods:

```bash
kubectl get pods

kubectl logs <pod name> -f
```

## Troubleshoot ROS Communication

To troubleshoot ROS messages, start a new bash shell in a running container:

```bash
# get name of running pods
kubectl get pods

# replace pod name
kubectl exec -it <pod name> /bin/bash
```

From the shell, run ROS tools as usual:

```bash
rostopic list
rostopic echo <topic name>

rosnode list
rosnode ping <node name>
```