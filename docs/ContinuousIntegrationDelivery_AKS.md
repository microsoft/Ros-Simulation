# Continuous Integration Continuous Delivery (CICD) for Azure Kubernetes Clusters (AKS)

## Continuous Integration (CI) using Visual Studio Team Service (VSTS)

* Under your project in VSTS, click "Build and release", and then click "New" to create a *build*

![Create CICD Pipeline](../images/cicd_createPipeline.png =1200x)

* Select "VSTS Git" and link this *build* to your code repository

![CICD Link Git](../images/cicd_linkGit.png =1200x)

* Select "Empty process" to create an empty template

![CICD Create Empty Process](../images/cicd_createEmptyProcess.png =1200x)

* Type in the *build* name

![CICD Set process](../images/cicd_setProcess.png =1200x)

* Install *helm* tool

![CICD Install Helm](../images/cicd_installHelm.png =1200x)

* Set *helm lint* to verify the chart for possible issues

![CICD Install Helm Chart](../images/cicd_installHelmChart.png =1200x)

![CICD Set Helm Lint ACS](../images/cicd_setHelmLint.png =1200x)

* Select "Docker" for the settings related to docker images

![CICD Create Docker](../images/cicd_createDocker.png =1200x)

* Build a docker image for *robot*

![CICD Build Robot Docker Image](../images/cicd_buildRobot.png =1200x)

* Push the *robot* docker image to Azure Container Registry

![CICD Push Robot Docker Image](../images/cicd_pushRobot.png =1200x)

* Build a docker image for *simulator*

![CICD Build Simulation Docker Image](../images/cicd_buildSimulation.png =1200x)

* Push the *simulator* docker image to Azure Container Registry

![CICD Push Simulator Docker Image](../images/cicd_pushSimulator.png =1200x)

* Copy the helm chart files from *build* to *release*

![CICD Copy Files](../images/cicd_copyFiles.png =1200x)

![CICD Copy File Settings](../images/cicd_copyFileSettings.png =1200x)

![CICD Publish](../images/cicd_publish.png =1200x)

![CICD Publish Setting](../images/cicd_publishSetting.png =1200x)

## Continuous Delivery (CD) using Visual Studio Team Service (VSTS)

* Under your project in VSTS, click "Build and release", and then click "Create a release pipeline" to create a *release*

![CICD Create Release](../images/cicd_release.png =800x)

* Select "Empty process" to create an empty template

![CICD Create Empty process](../images/cicd_emptyProcess.png =1200x)

* Set a name to your *release*

![CICD Name Release](../images/cicd_releaseName.png =1200x)

* Link the artifact from *build*

![CICD Link Artifact](../images/cicd_artifact.png =1200x)

* In the empty template, install "helm" tool to k8s cluster

![CICD Helm Install](../images/cicd_helmInstall.png =1200x)

* Set the "helm" and connect to your k8s cluster

![CICD Helm Set AKS](../images/cicd_helmInitSetAKS.png =1200x)

* Set "Helm upgrade" to deploy your helm chart to your k8s cluster

![CICD Helm Upgrade](../images/cicd_upgradeAKS.png =1200x)