# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

Param(
  [string]$nodeName
)

if (!$nodeName) {

    Write-Output "No nodeName given. Trying to find nodename with simulator agentpool name..."

    # If no node name is given, try to find simulator node.
    # This only works if simulator agentpool was created with acs-engine template.
    $nodeNames = $((kubectl get nodes | Select-String "simulator") -Split " ")
    
    if ($nodeNames) {
	    $nodeName = $nodeNames[0]
    }

    if (!$nodeName) {

        Write-Error "No simulator node given or found. Please provide a nodeName argument."
        return
    }

}

Write-Output "Labeling simulator node ${nodeName} as dedicated=simulator..."

kubectl label node $nodeName dedicated=simulator

Write-Output "Tainting simulator node ${nodeName} for dedicated simulator..."
kubectl taint nodes -l dedicated=simulator dedicated=simulator:NoSchedule
