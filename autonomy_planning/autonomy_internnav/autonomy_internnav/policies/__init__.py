"""Navigation policy inference wrappers."""

from autonomy_internnav.policies.base import InferenceResult, PolicyInference
from autonomy_internnav.policies.factory import create_policy_inference
from autonomy_internnav.policies.navdp_policy import NavDPPolicyInference

# Backward-compatible alias
NavDPInference = NavDPPolicyInference

__all__ = [
    'InferenceResult',
    'NavDPInference',
    'NavDPPolicyInference',
    'PolicyInference',
    'create_policy_inference',
]
