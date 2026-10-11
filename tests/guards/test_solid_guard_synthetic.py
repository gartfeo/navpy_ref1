"""Anti-loophole tests for the source-derived SOLID guard."""

from __future__ import annotations

import hashlib
from dataclasses import replace

from tests.guards.solid_architecture_guard import (
    apply_exact_exclusions,
    material_sources,
    scan_global_class_limits,
    scan_material_limits,
    scan_new_definition_annotations,
)
from tests.guards.solid_step1_relocation import Piece, Relocation


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


# Step-1 relocation fixtures: physical lines past the material module limit
# that leave the AST alone, so only materiality decides whether a file fails.
# The old tree is fictional (`attic`): the step-1 move tool neither rewrites
# these strings nor reports them as leftovers.
_PADDING = "\n" * 301
_WIDGET_OLD = "src/navpy/attic/vision/widget.py"
_WIDGET_NEW = "src/navpy/core/perception/widget.py"
_HELPERS_OLD = "src/navpy/attic/vision/helpers.py"
_HELPERS_NEW = "src/navpy/core/kernel/helpers.py"
_BUILDER_OLD = "src/navpy/attic/vision/builder.py"
_BUILDER_NEW = "src/navpy/core/perception/builder.py"
_RECORD_PART = "src/navpy/core/ports/record.py"
_MESSAGES_OLD = "src/navpy/attic/messages"
_HUB = "src/navpy/attic/vision/hub.py"
_PINNED_TREE = {
    "src/navpy/__init__.py": "",
    "src/navpy/attic/__init__.py": "",
    "src/navpy/attic/vision/__init__.py": (
        "from navpy.attic.vision.widget import Widget\n"
    ),
    _WIDGET_OLD: '''\
"""A widget."""
import logging

from .helpers import scale

_LOG = logging.getLogger("navpy.attic.vision.widget")


class Widget:
    def size(self, value):
        return scale(value)
''' + _PADDING,
    _HELPERS_OLD: "def scale(value):\n    return value\n\n\ndef shift(value):\n    return value\n",
    _BUILDER_OLD: '''\
"""A builder and the record it makes."""
from __future__ import annotations

import logging
from dataclasses import dataclass

_LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class Record:
    value: int

    def doubled(self):
        return self.value * 2


def build(value):
    return Record(value)


__all__ = ["Record", "build"]
''' + _PADDING,
    _HUB: (
        "from navpy.attic.vision.helpers import scale, shift\n\n\n"
        "class Hub:\n    pass\n\n\n"
        '__all__ = ["Hub", "scale", "shift"]\n'
    ) + _PADDING,
    f"{_MESSAGES_OLD}/__init__.py": "",
    f"{_MESSAGES_OLD}/msg_abc.py": "class MsgABC:\n    pass\n",
    f"{_MESSAGES_OLD}/types.py": "class MsgType:\n    pass\n",
}
_RELOCATION = Relocation(
    paths={
        _WIDGET_OLD: _WIDGET_NEW,
        _HELPERS_OLD: _HELPERS_NEW,
        _BUILDER_OLD: _BUILDER_NEW,
        f"{_MESSAGES_OLD}/__init__.py": "src/navpy/core/link/__init__.py",
        f"{_MESSAGES_OLD}/msg_abc.py": "src/navpy/core/link/msg_abc.py",
        f"{_MESSAGES_OLD}/types.py": "src/navpy/core/link/types.py",
    },
    modules={
        "navpy.attic.vision.widget": "navpy.core.perception.widget",
        "navpy.attic.vision.helpers": "navpy.core.kernel.helpers",
        "navpy.attic.vision.builder": "navpy.core.perception.builder",
        "navpy.attic.messages": "navpy.core.link",
        "navpy.attic.messages.msg_abc": "navpy.core.link.msg_abc",
        "navpy.attic.messages.types": "navpy.core.link.types",
    },
    symbol_modules={
        ("navpy.attic.vision.builder", "Record"): "navpy.core.ports.record",
    },
    parts={_RECORD_PART: (Piece(_BUILDER_OLD, frozenset({"Record"})),)},
    stripped={_HUB: frozenset({("scale", "navpy.attic.vision.helpers")})},
    directories={_MESSAGES_OLD: "src/navpy/core/link"},
)
# The widget at its step-1 home: relative import and logger name rewritten.
_WIDGET_MOVED = '''\
"""A widget."""
import logging

from navpy.core.kernel.helpers import scale

_LOG = logging.getLogger("navpy.core.perception.widget")


class Widget:
    def size(self, value):
        return scale(value)
''' + _PADDING
_RECORD_SPLIT = '''\
"""The record the builder makes."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Record:
    value: int

    def doubled(self):
        return self.value * 2
''' + _PADDING
_BUILDER_SPLIT = '''\
"""A builder."""
from __future__ import annotations

import logging

from navpy.core.ports.record import Record

_LOG = logging.getLogger(__name__)


def build(value):
    return Record(value)


__all__ = ["build"]
''' + _PADDING


def _relocated_material_codes(current: dict[str, str]) -> set[str]:
    return _codes(
        scan_material_limits(
            current, _PINNED_TREE, exclusions={}, relocation=_RELOCATION
        )
    )


def _relocated_new_definitions(current: dict[str, str]) -> set[str]:
    return {
        violation.owner
        for violation in scan_new_definition_annotations(
            current, _PINNED_TREE, exclusions={}, relocation=_RELOCATION
        )
    }


def test_pure_move_is_not_material_and_keeps_its_definitions() -> None:
    current = {_WIDGET_NEW: _WIDGET_MOVED}

    assert "material-module-lines" in _codes(
        scan_material_limits(current, _PINNED_TREE, exclusions={})
    )
    assert scan_new_definition_annotations(current, _PINNED_TREE, exclusions={})
    assert not material_sources(current, _PINNED_TREE, _RELOCATION)
    assert not _relocated_material_codes(current)
    assert not _relocated_new_definitions(current)


def test_real_change_in_a_moved_file_is_still_material() -> None:
    changed = _WIDGET_MOVED.replace("return scale(value)", "return scale(value) + 1")
    extra_import = _WIDGET_MOVED.replace(
        "import logging\n", "import json\nimport logging\n"
    )
    assert changed != _WIDGET_MOVED
    assert extra_import != _WIDGET_MOVED

    assert "material-module-lines" in _relocated_material_codes(
        {_WIDGET_NEW: changed}
    )
    assert "material-module-lines" in _relocated_material_codes(
        {_WIDGET_NEW: extra_import}
    )


def test_new_unannotated_definition_in_a_moved_file_is_reported() -> None:
    current = {
        _WIDGET_NEW: _WIDGET_MOVED.replace(
            "class Widget:",
            "def added(value):\n    return value\n\n\nclass Widget:",
        )
    }
    assert _relocated_new_definitions(current) == {"added"}


def test_facade_import_rewrite_is_not_material() -> None:
    legacy = (
        "from unittest import mock\n\n"
        "from navpy.attic.vision import Widget\n\n"
        'PATCH = mock.patch("navpy.attic.vision.helpers.scale")\n'
    ) + _PADDING
    rewritten = (
        "from unittest import mock\n\n"
        "from navpy.core.perception.widget import Widget\n\n"
        'PATCH = mock.patch("navpy.core.kernel.helpers.scale")\n'
    ) + _PADDING
    other_target = rewritten.replace("helpers.scale", "helpers.shift")
    pinned = {**_PINNED_TREE, "scripts/consumer.py": legacy}

    def codes(source: str, relocation: Relocation | None) -> set[str]:
        return _codes(
            scan_material_limits(
                {"scripts/consumer.py": source},
                pinned,
                exclusions={},
                relocation=relocation,
            )
        )

    assert "material-module-lines" in codes(rewritten, None)
    assert not codes(rewritten, _RELOCATION)
    assert "material-module-lines" in codes(other_target, _RELOCATION)


def test_pure_move_split_is_not_material() -> None:
    pinned = {
        **_PINNED_TREE,
        "scripts/importer.py": (
            "from navpy.attic.vision.builder import Record\n" + _PADDING
        ),
    }
    current = {
        _RECORD_PART: _RECORD_SPLIT,
        _BUILDER_NEW: _BUILDER_SPLIT,
        "scripts/importer.py": (
            "from navpy.core.ports.record import Record\n" + _PADDING
        ),
    }

    assert "material-module-lines" in _codes(
        scan_material_limits(current, pinned, exclusions={})
    )
    assert scan_new_definition_annotations(current, pinned, exclusions={})
    assert not scan_material_limits(
        current, pinned, exclusions={}, relocation=_RELOCATION
    )
    assert not scan_new_definition_annotations(
        current, pinned, exclusions={}, relocation=_RELOCATION
    )


def test_split_with_new_code_is_still_material_and_reported() -> None:
    changed = _RECORD_SPLIT.replace("self.value * 2", "self.value * 3")
    added = _RECORD_SPLIT.replace(
        "@dataclass(frozen=True)",
        "def build(value):\n    return value\n\n\n@dataclass(frozen=True)",
    )
    foreign_import = _RECORD_SPLIT.replace(
        "from dataclasses import dataclass\n",
        "from dataclasses import dataclass\n\n"
        "from navpy.core.perception.widget import Widget\n",
    )
    rest_changed = _BUILDER_SPLIT.replace("return Record(value)", "return None")
    assert len({changed, added, foreign_import, _RECORD_SPLIT}) == 4
    assert rest_changed != _BUILDER_SPLIT

    for source in (changed, added, foreign_import):
        assert "material-module-lines" in _relocated_material_codes(
            {_RECORD_PART: source}
        )
    assert "material-module-lines" in _relocated_material_codes(
        {_BUILDER_NEW: rest_changed}
    )
    # `build` was defined in the split source, but this part took only Record.
    assert _relocated_new_definitions({_RECORD_PART: added}) == {"build"}


def test_stripped_reexport_is_not_material_but_other_removals_are() -> None:
    stripped = (
        "from navpy.core.kernel.helpers import shift\n\n\n"
        "class Hub:\n    pass\n\n\n"
        '__all__ = ["Hub", "shift"]\n'
    ) + _PADDING
    unlisted = stripped.replace("shift", "scale")
    assert unlisted != stripped

    assert not _relocated_material_codes({_HUB: stripped})
    assert "material-module-lines" in _relocated_material_codes({_HUB: unlisted})


def test_message_paths_rewritten_by_hand_are_not_material() -> None:
    legacy = (
        "import os\n\n"
        'TEMPLATE = "from navpy.attic.messages.msg_abc import MsgABC"\n'
        'TYPES = os.path.join(ROOT, "src", "navpy", "attic", "messages", "types.py")\n'
        'WRAPPER = os.path.join(ROOT, "src", "navpy", "attic", "messages", name)\n'
    ) + _PADDING
    by_hand = (
        "import os\n\n"
        'TEMPLATE = "from navpy.core.link.msg_abc import MsgABC"\n'
        'TYPES = os.path.join(ROOT, "src", "navpy", "core", "link", "types.py")\n'
        'WRAPPER = os.path.join(ROOT, "src", "navpy", "core", "link", name)\n'
    ) + _PADDING
    elsewhere = by_hand.replace('"core", "link", name', '"core", "kernel", name')
    assert elsewhere != by_hand
    pinned = {**_PINNED_TREE, "scripts/add_message.py": legacy}

    def codes(source: str) -> set[str]:
        return _codes(
            scan_material_limits(
                {"scripts/add_message.py": source},
                pinned,
                exclusions={},
                relocation=_RELOCATION,
            )
        )

    assert not codes(by_hand)
    assert "material-module-lines" in codes(elsewhere)


def test_relocated_file_absent_at_the_base_keeps_new_file_semantics() -> None:
    relocation = Relocation(
        paths={"src/navpy/attic/vision/later.py": "src/navpy/core/later.py"},
        modules={"navpy.attic.vision.later": "navpy.core.later"},
    )
    current = {
        "src/navpy/core/later.py": "def later(value):\n    return value\n"
        + _PADDING
    }

    assert "material-module-lines" in _codes(
        scan_material_limits(
            current, _PINNED_TREE, exclusions={}, relocation=relocation
        )
    )
    assert {
        violation.owner
        for violation in scan_new_definition_annotations(
            current, _PINNED_TREE, exclusions={}, relocation=relocation
        )
    } == {"later"}


def test_split_that_drops_moves_or_keeps_claimed_code_is_material() -> None:
    record_only_docstring = (
        '"""The record the builder makes."""\nfrom __future__ import annotations\n'
        + _PADDING
    )
    builder_def = "\n\ndef build(value):\n    return Record(value)\n"
    record_with_build = _RECORD_SPLIT.replace(_PADDING, builder_def + _PADDING)
    rest_without_log = _BUILDER_SPLIT.replace(
        "_LOG = logging.getLogger(__name__)\n", ""
    )
    rest_keeping_record = _PINNED_TREE[_BUILDER_OLD]
    assert record_with_build != _RECORD_SPLIT
    assert rest_without_log != _BUILDER_SPLIT

    def material(part: str, rest: str) -> set[str]:
        current = {_RECORD_PART: part, _BUILDER_NEW: rest}
        return set(material_sources(current, _PINNED_TREE, _RELOCATION))

    assert material(_RECORD_SPLIT, _BUILDER_SPLIT) == set()
    assert material(record_only_docstring, _BUILDER_SPLIT) == {_RECORD_PART}
    assert material(record_with_build, _BUILDER_SPLIT) == {_RECORD_PART}
    assert material(_RECORD_SPLIT, rest_without_log) == {_BUILDER_NEW}
    assert material(_RECORD_SPLIT, rest_keeping_record) == {_BUILDER_NEW}
    # Before c02 makes the part, the whole source moving unchanged is old code.
    assert not material_sources(
        {_BUILDER_NEW: rest_keeping_record}, _PINNED_TREE, _RELOCATION
    )


def test_adding_or_dropping_future_annotations_is_material() -> None:
    widget_with_future = "from __future__ import annotations\n" + _WIDGET_MOVED
    record_without_future = _RECORD_SPLIT.replace(
        "from __future__ import annotations\n", ""
    )
    assert record_without_future != _RECORD_SPLIT

    assert "material-module-lines" in _relocated_material_codes(
        {_WIDGET_NEW: widget_with_future}
    )
    assert set(
        material_sources(
            {_RECORD_PART: record_without_future, _BUILDER_NEW: _BUILDER_SPLIT},
            _PINNED_TREE,
            _RELOCATION,
        )
    ) == {_RECORD_PART}


def test_only_path_builder_arguments_are_joined_as_one_path() -> None:
    legacy = (
        "import os\n\n"
        'TYPES = os.path.join(ROOT, "src", "navpy", "attic", "messages", "types.py")\n'
        'CALL = record("src", "navpy", "attic", "messages")\n'
    ) + _PADDING
    joined = legacy.replace(
        '"src", "navpy", "attic", "messages", "types.py"',
        '"src/navpy/core/link/types.py"',
    )
    other_call = legacy.replace(
        '"src", "navpy", "attic", "messages")', '"src/navpy/core/link")'
    )
    assert joined != legacy and other_call != legacy
    pinned = {**_PINNED_TREE, "scripts/paths.py": legacy}

    def material(source: str) -> bool:
        return bool(
            material_sources({"scripts/paths.py": source}, pinned, _RELOCATION)
        )

    assert not material(joined)
    assert material(other_call)


def test_a_split_that_drops_an_import_its_code_reads_is_material() -> None:
    part_without_dataclass = _RECORD_SPLIT.replace("from dataclasses import dataclass\n", "")
    rest_without_record = _BUILDER_SPLIT.replace(
        "from navpy.core.ports.record import Record\n", ""
    )
    assert part_without_dataclass != _RECORD_SPLIT
    assert rest_without_record != _BUILDER_SPLIT

    def material(part: str, rest: str) -> set[str]:
        current = {_RECORD_PART: part, _BUILDER_NEW: rest}
        return set(material_sources(current, _PINNED_TREE, _RELOCATION))

    assert material(part_without_dataclass, _BUILDER_SPLIT) == {_RECORD_PART}
    assert material(_RECORD_SPLIT, rest_without_record) == {_BUILDER_NEW}


def test_a_split_that_reorders_statements_is_material() -> None:
    reordered = _BUILDER_SPLIT.replace(
        "_LOG = logging.getLogger(__name__)\n\n\n"
        "def build(value):\n    return Record(value)\n",
        "def build(value):\n    return Record(value)\n\n\n"
        "_LOG = logging.getLogger(__name__)\n",
    )
    assert reordered != _BUILDER_SPLIT

    assert set(
        material_sources(
            {_RECORD_PART: _RECORD_SPLIT, _BUILDER_NEW: reordered},
            _PINNED_TREE,
            _RELOCATION,
        )
    ) == {_BUILDER_NEW}


def test_a_merged_part_holds_its_sources_in_the_listed_order() -> None:
    mixed = "src/navpy/core/ports/mixed.py"
    relocation = replace(
        _RELOCATION,
        symbol_modules={
            ("navpy.attic.vision.helpers", "shift"): "navpy.core.ports.mixed",
            ("navpy.attic.vision.builder", "Record"): "navpy.core.ports.mixed",
        },
        parts={mixed: (
            Piece(_HELPERS_OLD, frozenset({"shift"})),
            Piece(_BUILDER_OLD, frozenset({"Record"})),
        )},
    )
    shift = "def shift(value):\n    return value\n"
    record = _RECORD_SPLIT.split("from dataclasses import dataclass\n")[1]
    head = "from __future__ import annotations\n\nfrom dataclasses import dataclass\n\n\n"
    listed = head + shift + "\n\n" + record
    reversed_order = head + record.replace(_PADDING, "") + "\n\n" + shift + _PADDING

    def material(source: str) -> bool:
        return bool(material_sources({mixed: source}, _PINNED_TREE, relocation))

    assert not material(listed)
    assert material(reversed_order)


def test_a_split_source_left_whole_beside_its_new_part_is_material() -> None:
    """c02 made the part but left the source unchanged at its old path: the
    unchanged AST does not hide that Record is now defined twice."""
    unchanged_source = {_BUILDER_OLD: _PINNED_TREE[_BUILDER_OLD]}

    assert not material_sources(unchanged_source, _PINNED_TREE, _RELOCATION)
    assert set(
        material_sources(
            {**unchanged_source, _RECORD_PART: _RECORD_SPLIT}, _PINNED_TREE, _RELOCATION
        )
    ) == {_BUILDER_OLD}


def test_only_the_listed_module_level_re_export_is_stripped() -> None:
    hub = "src/navpy/attic/vision/hub2.py"
    pinned = {**_PINNED_TREE, hub: (
        "from navpy.attic.vision.helpers import scale, shift\n\n\n"
        "def size():\n    return 1\n\n\n"
        '__all__ = ["scale", "shift", "size"]\n'
    )}
    relocation = replace(
        _RELOCATION, stripped={hub: frozenset({("scale", "navpy.attic.vision.helpers")})}
    )
    stripped = (
        "from navpy.core.kernel.helpers import shift\n\n\n"
        "def size():\n    return 1\n\n\n"
        '__all__ = ["shift", "size"]\n'
    )
    nested = stripped.replace(
        "    return 1\n",
        "    from navpy.core.perception.builder import build as scale\n    return 1\n",
    )
    other_origin = stripped.replace(
        "\n\n\ndef size",
        "\nfrom navpy.core.perception.builder import build as scale\n\n\ndef size",
    )
    assert len({stripped, nested, other_origin}) == 3

    def material(source: str) -> bool:
        return bool(material_sources({hub: source}, pinned, relocation))

    assert not material(stripped)
    assert material(nested)
    assert material(other_origin)


def test_only_real_path_builders_have_their_segments_joined() -> None:
    legacy = (
        "import os\n"
        "from os import path as osp\n"
        "from pathlib import Path\n\n"
        "ROOT = Path(__file__).resolve().parents[1]\n"
        'A = os.path.join(base, "src", "navpy", "attic", "messages", "types.py")\n'
        'B = osp.join(base, "src", "navpy", "attic", "messages")\n'
        'C = ROOT / "src" / "navpy" / "attic" / "messages" / "types.py"\n'
        'D = ROOT.joinpath("src", "navpy", "attic", "messages")\n'
        'E = record.join("src", "navpy", "attic", "messages")\n'
        'F = base / "src" / "navpy" / "attic" / "messages"\n'
    )
    joined = (
        legacy.replace('"src", "navpy", "attic", "messages", "types.py"',
                       '"src/navpy/core/link/types.py"')
        .replace('osp.join(base, "src", "navpy", "attic", "messages")',
                 'osp.join(base, "src/navpy/core/link")')
        .replace('"src" / "navpy" / "attic" / "messages" / "types.py"',
                 '"src/navpy/core/link/types.py"')
        .replace('ROOT.joinpath("src", "navpy", "attic", "messages")',
                 'ROOT.joinpath("src", "navpy", "core", "link")')
    )
    other_join = joined.replace(
        'record.join("src", "navpy", "attic", "messages")', 'record.join("src/navpy/core/link")'
    )
    unknown_operand = joined.replace(
        'base / "src" / "navpy" / "attic" / "messages"', 'base / "src/navpy/core/link"'
    )
    assert len({legacy, joined, other_join, unknown_operand}) == 4
    pinned = {**_PINNED_TREE, "scripts/paths.py": legacy}

    def material(source: str) -> bool:
        return bool(
            material_sources({"scripts/paths.py": source}, pinned, _RELOCATION)
        )

    assert not material(joined)
    assert material(other_join)
    assert material(unknown_operand)
