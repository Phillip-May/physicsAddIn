"""Manifest node factories and the node-type enum shared by every package script."""

from __future__ import annotations

TYPE_CUSTOM = 2
TYPE_TRANSFORM = 7
TYPE_OPW6_ROBOT = 9
TYPE_ROBOT_LINK = 10
TYPE_ROBOT_TOOL = 11
TYPE_MESH_GEOMETRY = 12
TYPE_GANTRY_MECHANISM = 13
TYPE_DRAG_CHAIN_MECHANISM = 14

DEFAULT_COLOR = [0.72, 0.72, 0.72, 1.0]


def node(name: str, node_type: int, data: dict | None = None,
         color: list[float] | None = None, children: list[dict] | None = None) -> dict:
    return {"name": name, "color": color or DEFAULT_COLOR, "type": node_type,
            "data": data or {}, "children": children or []}


def mesh_node(name: str, source: str, color: list[float],
              mounting_holes: dict | None = None) -> dict:
    result = node(name, TYPE_MESH_GEOMETRY,
                  {"meshSource": source.replace("\\", "/")}, color)
    if mounting_holes:
        result["mountingHoles"] = mounting_holes
    return result


def transform_node(name: str, loc: list[float], mounting_holes: dict) -> dict:
    # Key order is part of the committed manifest bytes; keep it stable.
    return {"name": name, "color": DEFAULT_COLOR, "type": TYPE_TRANSFORM, "data": {},
            "loc": loc, "mountingHoles": mounting_holes, "children": []}


def root_node(name: str, children: list[dict], mounting_holes: dict,
              parametric_accessory: dict | None = None) -> dict:
    data = {"parametricAccessory": parametric_accessory} if parametric_accessory else {}
    return {"name": name, "color": DEFAULT_COLOR, "type": TYPE_TRANSFORM, "data": data,
            "mountingHoles": mounting_holes, "children": children}
