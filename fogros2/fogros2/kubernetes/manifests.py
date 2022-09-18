from typing import List


def wireguard(ns: str = "fogros2") -> List[dict]:
    """
    Creates a Wireguard node that can access the internal network.
    TODO: Netpol to only allow communications from the Wireguard pod to things in the same namespace.
    """
    namespace = {
        "apiVersion": "v1",
        "kind": "Namespace",
        "metadata": {"name": ns, "labels": {"name": ns}},
    }

    pod = {
        "apiVersion": "v1",
        "kind": "Pod",
        "metadata": {
            "name": "wireguard",
            "namespace": ns,
            "labels": {"app": "wireguard"},
        },
        "spec": {
            "containers": [
                {
                    "name": "wireguard",
                    "image": "ghcr.io/linuxserver/wireguard",
                    "envFrom": [{"configMapRef": {"name": "wireguard"}}],
                    "securityContext": {
                        "capabilities": {"add": ["NET_ADMIN", "SYS_MODULE"]},
                        "privileged": True,
                    },
                    "ports": [{"containerPort": 51820, "protocol": "UDP"}],
                    "resources": {
                        "requests": {"memory": "64Mi", "cpu": "100m"},
                        "limits": {"memory": "128Mi", "cpu": "200m"},
                    },
                }
            ],
        },
    }

    configmap = {
        "apiVersion": "v1",
        "kind": "ConfigMap",
        "metadata": {"name": "wireguard", "namespace": ns},
        "data": {
            "PUID": "1000",
            "PGID": "1000",
            "PEERDNS": "10.96.62.130",
            "TZ": "America/Los_Angeles",
            "SERVERPORT": "51820",
            "PEERS": "3",
            "ALLOWEDIPS": "10.0.0.0/8",
            "INTERNAL_SUBNET": "10.13.13.0",
        },
    }

    svc = {
        "kind": "Service",
        "apiVersion": "v1",
        "metadata": {
            "labels": {"app": "wireguard"},
            "name": "wireguard",
            "namespace": ns,
        },
        "spec": {
            "type": "LoadBalancer",
            "ports": [
                {
                    "port": 51820,
                    "targetPort": 51820,
                    "name": "wg",
                    "protocol": "UDP",
                }
            ],
            "selector": {"app": "wireguard"},
        },
    }

    return [namespace, pod, configmap, svc]
