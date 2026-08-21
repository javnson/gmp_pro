"""Facility-registry selection and transitive dependency helpers."""

from __future__ import annotations


class RegistrySelectionError(ValueError):
    """Raised when a project selects an invalid Facility module graph."""


def resolve_selected_modules(registry: dict, local_config: dict) -> list[tuple[str, str]]:
    """Return selected modules and their transitive dependencies in stable order."""
    modules = registry.get("modules", {})
    pending: list[tuple[str, str]] = []
    for item in local_config.get("selected_modules", []):
        root = item.get("root")
        module = item.get("module")
        if not isinstance(root, str) or not root or not isinstance(module, str) or not module:
            raise RegistrySelectionError(f"Invalid selected module entry: {item!r}")
        pending.append((root, module))

    resolved: list[tuple[str, str]] = []
    visited: set[tuple[str, str]] = set()
    while pending:
        root, module = pending.pop(0)
        key = (root, module)
        if key in visited:
            continue
        module_data = modules.get(root, {}).get(module)
        if not isinstance(module_data, dict):
            prefix = module + "|"
            descendants = [
                candidate
                for candidate, candidate_data in modules.get(root, {}).items()
                if candidate.startswith(prefix)
                and isinstance(candidate_data, dict)
                and candidate_data.get("type") == "module"
            ]
            if descendants:
                pending[0:0] = [(root, candidate) for candidate in descendants]
                continue
            raise RegistrySelectionError(f"Facility module is not registered: {root}|{module}")
        visited.add(key)
        resolved.append(key)
        for dependency in module_data.get("depends_on", []):
            if not isinstance(dependency, str) or "|" not in dependency:
                raise RegistrySelectionError(
                    f"Invalid dependency {dependency!r} declared by {root}|{module}"
                )
            dependency_root, dependency_module = dependency.split("|", 1)
            pending.append((dependency_root, dependency_module))

    selected_csps = [module for root, module in resolved if root == "csp"]
    if len(selected_csps) > 1:
        formatted = ", ".join(f"csp|{module}" for module in selected_csps)
        raise RegistrySelectionError(
            "A project may resolve at most one concrete CSP; "
            f"multiple CSP modules were selected: {formatted}"
        )
    return resolved
