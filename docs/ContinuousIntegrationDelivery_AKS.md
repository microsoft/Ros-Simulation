# Continuous Integration Continuous Delivery (CICD) for Azure Kubernetes Clusters (AKS)

## Continuous Integration (CI) using Visual Studio Team Service (VSTS)

* Under your project in VSTS, click "Build and release", and then click "New" to create a *build*

<img src="../images/cicd_createPipeline.png" alt="Create CICD Pipeline" width="1200" />

* Select "VSTS Git" and link this *build* to your code repository

![CICD Link Git](../images/cicd_linkGit.png =1200x)

* Select "Empty process" to create an empty template

<img src="../images/cicd_createEmptyProcess.png" alt="CICD Create Empty Process" width="1200" />

* Type in the *build* name

![CICD Set process](../images/cicd_setProcess.png =1200x)

* Install *helm* tool

![CICD Install Helm](../images/cicd_installHelm.png =1200x)

* Set *helm lint* to verify the chart for possible issues

![CICD Install Helm Chart](../images/cicd_installHelmChart.png =1200x)

![CICD Set Helm Lint ACS](../images/cicd_setHelmLint.png =1200x)

* Select "Docker" for the settings related to docker images

<img src="../images/cicd_createDocker.png" alt="CICD Create Docker" width="1200" />

* Build a docker image for *robot*

<img src="../images/cicd_buildRobot.png" alt="CICD Build Robot Docker Image" width="1200" />

* Push the *robot* docker image to Azure Container Registry

![CICD Push Robot Docker Image](../images/cicd_pushRobot.png =1200x)

* Build a docker image for *simulator*

<img src="../images/cicd_buildSimulation.png" alt="CICD Build Simulation Docker Image" width="1200" />

* Push the *simulator* docker image to Azure Container Registry

![CICD Push Simulator Docker Image](../images/cicd_pushSimulator.png =1200x)

* Copy the helm chart files from *build* to *release*

<img src="../images/cicd_copyFiles.png" alt="CICD Copy Files" width="1200" />

<img src="../images/cicd_copyFileSettings.png" alt="CICD Copy File Settings" width="1200" />

![CICD Publish](../images/cicd_publish.png =1200x)

![CICD Publish Setting](../images/cicd_publishSetting.png =1200x)

## Continuous Delivery (CD) using Visual Studio Team Service (VSTS)

* Under your project in VSTS, click "Build and release", and then click "Create a release pipeline" to create a *release*

![CICD Create Release](../images/cicd_release.png =800x)

* Select "Empty process" to create an empty template

<img src="../images/cicd_emptyProcess.png" alt="CICD Create Empty process" width="1200" />

* Set a name to your *release*

![CICD Name Release](../images/cicd_releaseName.png =1200x)

* Link the artifact from *build*

<img src="../images/cicd_artifact.png" alt="CICD Link Artifact" width="1200" />

* In the empty template, install "helm" tool to k8s cluster

![CICD Helm Install](../images/cicd_helmInstall.png =1200x)

* Set the "helm" and connect to your k8s cluster

<img src="../images/cicd_helmInitSetAKS.png" alt="CICD Helm Set AKS" width="1200" />

* Set "Helm upgrade" to deploy your helm chart to your k8s cluster

![CICD Helm Upgrade](../images/cicd_upgradeAKS.png =1200x)