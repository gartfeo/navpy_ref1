"""Anti-loophole tests for the source-derived SOLID guard."""

from __future__ import annotations

import hashlib

from tests.guards.solid_architecture_guard import (
    apply_exact_exclusions,
    material_sources,
    scan_global_class_limits,
    scan_material_limits,
    scan_new_definition_annotations,
)


def _codes(violations) -> set[str]:
    return {violation.code for violation in violations}


def test_class_limits_cover_decorators_nested_classes_and_direct_methods() -> None:
    giant = "@marker\nclass Giant:\n" + "\n".join(
        f"    value_{index} = {index}" for index in range(199)
    )
    methods = "\n".join(
        f"            def method_{index}(self): ..." for index in range(16)
    )
    nested = f"class Outer:\n    def factory(self):\n        class Hidden:\n{methods}\n"
    violations = scan_global_class_limits(
        {"src/navpy/giant.py": giant, "src/navpy/nested.py": nested},
        exclusions={},
    )
    assert "class-lines" in _codes(violations)
    assert any(
        violation.code == "direct-methods"
        and violation.owner.endswith("Hidden")
        for violation in violations
    )


def test_imported_facets_count_toward_effective_method_limit() -> None:
    methods = "\n".join(
        f"    def method_{index}(self): ..." for index in range(16)
    )
    sources = {
        "src/navpy/facets.py": f"class ExtraFacet:\n{methods}\n",
        "src/navpy/facade.py": """\
from navpy.facets import ExtraFacet

class Facade(ExtraFacet):
    def __init__(self):
        self._parts = object()
""",
    }

    violations = scan_global_class_limits(sources, exclusions={})

    assert any(
        violation.code == "effective-methods"
        and violation.owner == "Facade"
        for violation in violations
    )


def test_aliased_imported_facets_cannot_evade_effective_method_limit() -> None:
    methods = "\n".join(
        f"    def method_{index}(self): ..." for index in range(16)
    )
    sources = {
        "src/navpy/facets.py": f"class ExtraFacet:\n{methods}\n",
        "src/navpy/facade.py": """\
from navpy.facets import ExtraFacet as Base

class Facade(Base):
    def __init__(self):
        self._parts = object()
""",
    }

    violations = scan_global_class_limits(sources, exclusions={})

    assert any(
        violation.code == "effective-methods"
        and violation.owner == "Facade"
        for violation in violations
    )


def test_compatibility_facade_allowance_is_exact_and_shape_bounded() -> None:
    methods = "\n".join(
        f"    def method_{index}(self): ..." for index in range(15)
    )
    controller = f"""\
class VisionController:
    def __init__(self):
        self._ports = object()
{methods}
"""
    exact = {"src/navpy/modules/vision/vision_controller.py": controller}
    assert not scan_global_class_limits(exact, exclusions={})

    wrong_path = {"src/navpy/vision_controller.py": controller}
    assert "direct-methods" in _codes(
        scan_global_class_limits(wrong_path, exclusions={})
    )

    wrong_shape = controller.replace("self._ports", "self._hidden")
    assert "direct-methods" in _codes(
        scan_global_class_limits(
            {"src/navpy/modules/vision/vision_controller.py": wrong_shape},
            exclusions={},
        )
    )

    extra_shape = controller.replace(
        "self._ports = object()",
        "self._ports = object()\n        self._hidden = object()",
    )
    assert "direct-methods" in _codes(
        scan_global_class_limits(
            {"src/navpy/modules/vision/vision_controller.py": extra_shape},
            exclusions={},
        )
    )


def test_inherited_facet_fields_count_toward_facade_limit() -> None:
    first_fields = "\n".join(
        f"        self.first_{index} = {index}" for index in range(12)
    )
    second_fields = "\n".join(
        f"        self.second_{index} = {index}" for index in range(12)
    )
    sources = {
        "src/navpy/facets.py": f"""\
class FirstFacet:
    def __init__(self):
{first_fields}

class SecondFacet:
    def initialize(self):
{second_fields}
""",
        "src/navpy/facade.py": """\
from navpy.facets import FirstFacet, SecondFacet

class Facade(FirstFacet, SecondFacet):
    pass
""",
    }

    violations = scan_global_class_limits(sources, exclusions={})

    assert any(
        violation.code == "owned-fields" and violation.owner == "Facade"
        for violation in violations
    )


def test_owned_fields_include_literal_dynamic_write_forms() -> None:
    direct = "\n".join(
        f"        self.field_{index} = {index}" for index in range(9)
    )
    source = f"""\
class Owner:
    def __init__(self):
{direct}
        setattr(self, "field_9", 9)
        object.__setattr__(self, "field_10", 10)
        self.__dict__["field_11"] = 11
        vars(self)["field_12"] = 12
"""
    violations = scan_global_class_limits(
        {"src/navpy/owner.py": source}, exclusions={}
    )
    assert "owned-fields" in _codes(violations)


def test_dynamic_field_names_fail_instead_of_evading_the_guard() -> None:
    source = """\
class Owner:
    def install(self, name, value):
        self.__setattr__(name, value)
"""
    violations = scan_global_class_limits(
        {"src/navpy/owner.py": source}, exclusions={}
    )
    assert "dynamic-field-write" in _codes(violations)


def test_class_constants_are_not_instance_owned_fields() -> None:
    source = """\
class Constants:
    RED = 1
    BLUE: int = 2
"""
    assert not scan_global_class_limits(
        {"src/navpy/constants.py": source}, exclusions={}
    )


def test_passive_record_exception_is_exact_frozen_and_bounded() -> None:
    fields = "\n".join(f"    field_{index}: int" for index in range(13))
    record = f"@dataclass(frozen=True)\nclass NavigationSample:\n{fields}\n"
    exact = {"src/navpy/logger/navigation_sample.py": record}
    assert not scan_global_class_limits(exact, exclusions={})

    wrong_path = {"src/navpy/other.py": record}
    assert "owned-fields" in _codes(
        scan_global_class_limits(wrong_path, exclusions={})
    )

    behavioral = record + "    def mutate(self): ...\n"
    assert "owned-fields" in _codes(
        scan_global_class_limits(
            {"src/navpy/logger/navigation_sample.py": behavioral},
            exclusions={},
        )
    )

    oversized = record.replace(
        "    field_12: int", "    field_12: int\n    field_13: int\n    field_14: int\n    field_15: int\n    field_16: int"
    )
    assert "owned-fields" in _codes(
        scan_global_class_limits(
            {"src/navpy/logger/navigation_sample.py": oversized},
            exclusions={},
        )
    )


def test_tk_composition_allowance_cannot_broaden() -> None:
    fields = "\n".join(f"        self.field_{index} = {index}" for index in range(13))
    launcher = f"class Launcher(tk.Tk):\n    def __init__(self):\n{fields}\n"
    exact = {"src/navpy/main_gui.py": launcher}
    assert not scan_global_class_limits(exact, exclusions={})

    wrong_path = {"src/navpy/copied_launcher.py": launcher}
    assert "owned-fields" in _codes(
        scan_global_class_limits(wrong_path, exclusions={})
    )

    wrong_base = launcher.replace("tk.Tk", "object")
    assert "owned-fields" in _codes(
        scan_global_class_limits(
            {"src/navpy/main_gui.py": wrong_base}, exclusions={}
        )
    )

    larger = launcher.replace(
        "        self.field_12 = 12",
        "        self.field_12 = 12\n        self.field_13 = 13",
    )
    assert "owned-fields" in _codes(
        scan_global_class_limits(
            {"src/navpy/main_gui.py": larger}, exclusions={}
        )
    )


def test_exclusions_require_both_exact_path_and_sha256() -> None:
    source = "class Vendor: ...\n"
    digest = hashlib.sha256(source.encode("utf-8")).hexdigest()
    exclusions = {"src/navpy/vendor.py": digest}

    kept, violations = apply_exact_exclusions(
        {"src/navpy/vendor.py": source}, exclusions
    )
    assert not kept
    assert not violations

    kept, violations = apply_exact_exclusions(
        {"src/navpy/vendor.py": source + "# changed\n"}, exclusions
    )
    assert kept
    assert "excluded-source-sha256" in _codes(violations)

    kept, violations = apply_exact_exclusions(
        {"src/navpy/copied.py": source}, exclusions
    )
    assert kept
    assert not violations


def test_material_scope_uses_ast_difference_not_a_manifest() -> None:
    legacy = "def old():\n    return 1\n" + "\n" * 301
    reformatted = "def old():\n\n    return 1\n" + "\n" * 300
    changed = legacy.replace("return 1", "return 2")
    assert not material_sources({"scripts/tool.py": reformatted}, {"scripts/tool.py": legacy})
    assert "material-module-lines" in _codes(
        scan_material_limits(
            {"scripts/tool.py": changed},
            {"scripts/tool.py": legacy},
            exclusions={},
        )
    )


def test_future_annotations_only_change_does_not_make_legacy_code_material() -> None:
    pinned = '"""module"""\n\nclass Legacy:\n    pass\n'
    current = (
        '"""module"""\n\nfrom __future__ import annotations\n\n'
        "class Legacy:\n    pass\n"
    )

    assert not material_sources(
        {"tools/legacy.py": current},
        {"tools/legacy.py": pinned},
    )


def test_material_top_level_function_limit_detects_new_long_function() -> None:
    body = "\n".join(f"    value_{index} = {index}" for index in range(150))
    source = f"def too_long() -> None:\n{body}\n"
    violations = scan_material_limits(
        {"src/navpy/new_leaf.py": source}, {}, exclusions={}
    )
    assert "top-level-function-lines" in _codes(violations)


def test_annotations_apply_only_to_new_qualified_definitions() -> None:
    pinned = {"src/navpy/example.py": "def legacy(value):\n    return value\n"}
    current = {
        "src/navpy/example.py": """\
def legacy(value):
    return value + 1

def added(value, *items, **options):
    return value
"""
    }
    violations = scan_new_definition_annotations(
        current, pinned, exclusions={}
    )
    assert {violation.owner for violation in violations} == {"added"}
    assert _codes(violations) == {
        "missing-parameter-annotations",
        "missing-return-annotation",
    }


def test_self_and_cls_are_the_only_implicit_annotation_exceptions() -> None:
    source = """\
class Factory:
    def build(self, value) -> None: ...

    @classmethod
    def make(cls) -> None: ...
"""
    violations = scan_new_definition_annotations(
        {"src/navpy/factory.py": source}, {}, exclusions={}
    )
    assert len(violations) == 1
    assert violations[0].detail.endswith("['value']")
