# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

Param(
  [string]$secretName="image-secret",
  [string]$acrLoginServer,
  [string]$appId,
  [string]$password,
  [string]$email="foo.bar@contoso.com"
)

kubectl create secret docker-registry $secretName --docker-server $acrLoginServer --docker-username $appId --docker-password $password --docker-email $email