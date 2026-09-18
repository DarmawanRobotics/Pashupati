import xml.etree.ElementTree as ET

import py_trees

from robot_mission.behaviors.registry import BEHAVIORS

COMPOSITES = {
    'Sequence': py_trees.composites.Sequence,
    'Selector': py_trees.composites.Selector,
}

PARALLEL_POLICIES = {
    'SuccessOnOne': py_trees.common.ParallelPolicy.SuccessOnOne,
    'SuccessOnAll': py_trees.common.ParallelPolicy.SuccessOnAll,
}

DECORATORS = {
    'Retry': py_trees.decorators.Retry,
    'Repeat': py_trees.decorators.Repeat,
    'Inverter': py_trees.decorators.Inverter,
}


def parse_bool(value: str) -> bool:
    """Parse an XML attribute string as a boolean."""
    return value.strip().lower() in ('1', 'true', 'yes')


def build_node(element: ET.Element, node) -> py_trees.behaviour.Behaviour:
    """Recursively build a py_trees behaviour from one XML element."""
    tag = element.tag
    name = element.get('name', tag)

    if tag in COMPOSITES:
        children = [build_node(child, node) for child in element]
        memory = parse_bool(element.get('memory', 'true'))
        return COMPOSITES[tag](name=name, memory=memory, children=children)

    if tag == 'Parallel':
        children = [build_node(child, node) for child in element]
        policy = PARALLEL_POLICIES[element.get('policy', 'SuccessOnOne')]()
        return py_trees.composites.Parallel(name=name, policy=policy, children=children)

    if tag in DECORATORS:
        if len(element) != 1:
            raise ValueError(f'{tag} decorator must have exactly one child, got {len(element)}')
        child = build_node(element[0], node)
        if tag == 'Retry':
            return py_trees.decorators.Retry(name=name, child=child, num_failures=int(element.get('num_failures', '1')))
        if tag == 'Repeat':
            return py_trees.decorators.Repeat(name=name, child=child, num_success=int(element.get('num_success', '-1')))
        return DECORATORS[tag](name=name, child=child)

    if tag in BEHAVIORS:
        return BEHAVIORS[tag](node, name=name)

    raise ValueError(f'unknown tree element: {tag!r}')


def load_tree(xml_path: str, node) -> py_trees.behaviour.Behaviour:
    """Parse an XML tree definition file and build the corresponding py_trees root behaviour."""
    root_element = ET.parse(xml_path).getroot()
    if len(root_element) != 1:
        raise ValueError('BehaviorTree root must have exactly one child')
    return build_node(root_element[0], node)
