"""Preserve YAML comments and commit related configuration files together."""

import json
import os
from pathlib import Path
import tempfile

import yaml


def replace_value(text, keys, value):
    values = value
    for key in reversed(keys):
        values = {key: values}
    return replace_parameters(text, values)


def replace_parameters(text, values):
    edits = []

    def visit(node, updates, field=None):
        if isinstance(node, yaml.MappingNode) and isinstance(updates, dict):
            existing = {key.value: (key, child) for key, child in node.value}
            missing = {key: value for key, value in updates.items() if key not in existing}
            if missing and node.flow_style:
                merged = yaml.safe_load(text[node.start_mark.index:node.end_mark.index])
                for key, value in updates.items():
                    merged[key] = value
                edits.append((node.start_mark.index, node.end_mark.index, json.dumps(merged, ensure_ascii=False)))
                return
            if missing:
                start = sum(len(line) for line in text.splitlines(keepends=True)[:node.start_mark.line])
                indent = ' '*node.start_mark.column
                addition = ''.join(f'{indent}{key}: {json.dumps(value, ensure_ascii=False)}\n' for key, value in missing.items())
                edits.append((start, start, addition))
            for key, value in updates.items():
                if key in existing:
                    child_field, child = existing[key]
                    visit(child, value, child_field)
            return
        if yaml.safe_load(text[node.start_mark.index:node.end_mark.index]) == updates:
            return
        replacement = json.dumps(updates, ensure_ascii=False)
        if field is not None and node.start_mark.line > field.start_mark.line and node.start_mark.column <= field.start_mark.column:
            replacement = ' '*(field.start_mark.column+2-node.start_mark.column)+replacement
        if node.end_mark.line > node.start_mark.line:
            replacement += '\n'+' '*node.end_mark.column
        edits.append((node.start_mark.index, node.end_mark.index, replacement))

    try:
        document = yaml.compose(text)
    except yaml.YAMLError as error:
        raise ValueError(f'配置YAML格式错误，未保存参数：{error}') from error
    if not isinstance(document, yaml.MappingNode) or not set(values) <= {key.value for key, _ in document.value}:
        raise ValueError('配置缺少对应区域映射')
    visit(document, values)
    for start, end, replacement in sorted(edits, key=lambda item: item[:2], reverse=True):
        text = text[:start]+replacement+text[end:]
    return text


def write_configs(changes):
    prepared, committed = [], []
    try:
        for path, (original, text) in changes.items():
            with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=path.parent,
                                             prefix=path.name+'.save.', delete=False) as stream:
                temporary = Path(stream.name)
                prepared.append((path, original, temporary))
                stream.write(text)
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(temporary, path.stat().st_mode & 0o777)
        for path, original, _ in prepared:
            if path.read_text(encoding='utf-8') != original:
                raise ValueError(f'配置刚被其他程序修改，请刷新后重试：{path}')
        for path, original, temporary in prepared:
            os.replace(temporary, path)
            committed.append((path, original))
    except Exception:
        for path, original in reversed(committed):
            path.write_text(original, encoding='utf-8')
        raise
    finally:
        for _, _, temporary in prepared:
            temporary.unlink(missing_ok=True)
