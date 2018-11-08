# Environment Setup

The ARGoS simulation runs in containers using Kubernetes as an orchestrator.

Since it is a container-based architecture, the simulation can be run in the cloud or in a local [Minikube](https://kubernetes.io/docs/setup/minikube/#installation)
cluster, which is supported on multiple platforms.

Below are the required and optional tooling for running the simulation.

## *Required Tooling*

---

### Command-line Tools

- [Docker](https://docs.docker.com/install/)
  - [Windows Install](https://docs.docker.com/docker-for-windows/install/#what-to-know-before-you-install)
  - [Ubuntu Install](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
- [Kubectl](https://kubernetes.io/docs/tasks/tools/install-kubectl/)
- [Helm](https://docs.helm.sh/using_helm/#installing-helm)

** match the version of helm to the version in your CI/CD pipeline, if one exists

- [Azure CLI](https://docs.microsoft.com/en-us/cli/azure/install-azure-cli?view=azure-cli-latest)
- [ACS-Engine](https://github.com/Azure/acs-engine/blob/master/docs/acsengine.md)

To test the installs, open a terminal/bash/command window and type the commands below.

There should be help text displayed after each command:

```console
> docker

> kubectl

> helm

> az

> acs-engine
```

### Azure Resources

- [Azure Container Registry](https://azure.microsoft.com/en-us/services/container-registry/) (or [Docker Registry](https://docs.docker.com/registry/))

## *Optional Tooling*

---

### Visualization

If using visualization, then a vnc client is needed to see the remote desktop. We recommend:

- [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/)

If running locally without docker (not recommended), then an Ubuntu OS is required:

- [Ubuntu 16.04 LTS](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0)
- [ARGoS](https://www.argos-sim.info/)
- [ROS](http://wiki.ros.org/kinetic/Installation)

** See the [LocalSetupAndTesting](./LocalSetupAndTesting.md) doc for more detailed instructions on running locally.
