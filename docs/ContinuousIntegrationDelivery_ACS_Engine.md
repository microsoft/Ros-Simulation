# Continuous Integration Continuous Delivery (CICD) for Azure Container Service Engine (ACS Engine)

## Continuous Integration (CI) using Visual Studio Team Service (VSTS)

* Under your project in VSTS, click "Build and release", and then click "New" to create a *build*

<img src="../images/cicd_createPipeline.png" alt="Create CICD Pipeline" width="1200" />

* Select "VSTS Git" and link this *build* to your code repository

<img src="../images/cicd_linkGit.png" alt="CICD Link Git" width="1200">

* Select "Empty process" to create an empty template

<img src="../images/cicd_createEmptyProcess.png" alt="CICD Create Empty Process" width="1200" />

* Type in the *build* name

<img src="../images/cicd_setProcess.png" alt="CICD Set process" width="1200">

* Install *helm* tool

<img src="../images/cicd_installHelm.png" alt="CICD Install Helm" width="1200">

* Set *helm lint* to verify the chart for possible issues. For ACS Engine cluster, we utilize Kubernetes Service Connection to connect to the k8s cluster.

<img src="../images/cicd_installHelmChart.png" alt="CICD Install Helm Chart" width="1200">

<img src="../images/cicd_setHelmLintACS.png" alt="CICD Set Helm Lint ACS" width="1200">

* Set the k8s connection name and copy "Server URL" and "KubeConfig" from your "kubeconfig.<cluster_region>.json" file. Click "Verify connection" to verify the k8s connection

<img src="../images/cicd_aksConnection.png" alt="CICD AKS Connection" width="600" />

* Select "Docker" for the settings related to docker images

<img src="../images/cicd_createDocker.png" alt="CICD Create Docker" width="1200" />

* Build a docker image for *robot*

<img src="../images/cicd_buildRobot.png" alt="CICD Build Robot Docker Image" width="1200" />

* Push the *robot* docker image to Azure Container Registry

<img src="../images/cicd_pushRobot.png" alt="CICD Push Robot Docker Image" width="1200">

* Build a docker image for *simulator*

<img src="../images/cicd_buildSimulation.png" alt="CICD Build Simulation Docker Image" width="1200" />

* Push the *simulator* docker image to Azure Container Registry

<img src="../images/cicd_pushSimulator.png" alt="CICD Push Simulator Docker Image" width="1200">

* Copy the helm chart files from *build* to *release*

<img src="../images/cicd_copyFiles.png" alt="CICD Copy Files" width="1200" />

<img src="../images/cicd_copyFileSettings.png" alt="CICD Copy File Settings" width="1200" />

<img src="../images/cicd_publish.png" alt="CICD Publish" width="1200">

<img src="../images/cicd_publishSetting.png" alt="CICD Publish Setting" width="1200">

## Continuous Delivery (CD) using Visual Studio Team Service (VSTS)

* Under your project in VSTS, click "Build and release", and then click "Create a release pipeline" to create a *release*

<img src="../images/cicd_release.png" alt="CICD Create Release" width="800">

* Select "Empty process" to create an empty template

<img src="../images/cicd_emptyProcess.png" alt="CICD Create Empty process" width="1200" />

* Set a name to your *release*

<img src="../images/cicd_releaseName.png" alt="CICD Name Release" width="1200">

* Link the artifact from *build*

<img src="../images/cicd_artifact.png" alt="CICD Link Artifact" width="1200" />

* Set the "helm init" to install helm and tiller on your k8s cluster

<img src="../images/cicd_helmInitSet.png" alt="CICD Helm Install" width="1200" />

* Set "Helm upgrade" to deploy your helm chart to your k8s cluster

<img src="../images/cicd_upgrade.png" alt="CICD Helm Upgrade" width="1200">