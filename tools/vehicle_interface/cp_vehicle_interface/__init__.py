"""CP Models vehicle interface: contract loading, FMU conformance checking, descriptors."""
from .check import STANDARD_PROFILE, Report, check
from .contract import Contract, load_contract
from .descriptor import PrepareError, prepare
from .metadata import Metadata, MetadataError, parse, read_model_description

__all__ = ["STANDARD_PROFILE", "Contract", "Metadata", "MetadataError", "PrepareError",
           "Report", "check", "load_contract", "parse", "prepare", "read_model_description"]
