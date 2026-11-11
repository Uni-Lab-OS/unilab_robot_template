"""Brand catalog; optional backends load only when explicitly selected."""
from unilab_robot_contracts.brand_catalog import BrandCatalog

catalog = BrandCatalog(__file__)
models = catalog.models
model = catalog.model
build_moveit_model = catalog.build_moveit_model
resolve_asset = catalog.asset
source_profiles = catalog.source_profiles
module_definition = catalog.module_definition
compile_asset_model = catalog.compile_asset_model
fetch_asset = catalog.fetch_asset

hardware_candidates = catalog.hardware_candidates
create_hardware_driver = catalog.create_hardware_driver
