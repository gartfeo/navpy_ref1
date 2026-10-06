"""Offline integrity/association gate; complete capture does not certify delivery."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Optional


def reject_nonfinite(value: str) -> None:
    raise ValueError(f'nonfinite JSON number: {value}')


def check_projection_capture(path: Path, *, expected_projections: Optional[int] = None) -> dict[str, Any]:
    reasons: list[str] = []
    try:
        manifest = json.loads(path.with_suffix('.manifest.json').read_text(encoding='utf-8'),
                              parse_constant=reject_nonfinite)
        if not manifest['complete'] or not manifest['closed'] or manifest['failed'] or manifest['failure'] or manifest['budget_exhausted']:
            reasons.append('manifest records an incomplete capture')
        if any(type(manifest[key]) is not int or manifest[key] < 0 for key in ('submitted','written','failed','bytes_written')):
            reasons.append('invalid manifest counts')
        if manifest['first_sequence'] != (1 if manifest['written'] else None) or manifest['last_sequence'] != (manifest['written'] or None):
            reasons.append('manifest sequence range disagrees')
        if manifest['submitted'] != manifest['written'] or manifest['written'] > manifest['budget']['max_records']:
            reasons.append('record counts or budget disagree')
        size = path.stat().st_size
        if size != manifest['bytes_written'] or size > manifest['budget']['max_bytes']:
            reasons.append('byte count or budget disagrees')
        if reasons:
            return dict(valid=False,reasons=sorted(set(reasons)),manifest=manifest)
        digest = hashlib.sha256()
        frames: dict[int, dict[str, Any]] = {}
        publications: set[int] = set()
        projections = count = 0
        with path.open('rb') as stream:
            for count, data in enumerate(stream, 1):
                digest.update(data)
                row = json.loads(data.decode('utf-8'), parse_constant=reject_nonfinite)
                if type(row['record_sequence']) is not int or row['record_sequence'] != count:
                    reasons.append('noncontiguous file sequence')
                if any(row[key] != manifest[key] for key in ('run_id','pid','vehicle_sysid','detector_id','detector_name')):
                    reasons.append('run identity disagreement')
                frame = row['frame']
                identity = {key: frame[key] for key in ('frame_id','epoch','generation','timestamp_s','declared_epoch','receipt_timestamp_s')}
                if frames.setdefault(frame['frame_id'], identity) != identity:
                    reasons.append('frame identity disagreement')
                if row['kind'] == 'projection':
                    projections += 1
                    if frame['frame_id'] in publications:
                        reasons.append('projection after frame publication record')
                    evidence = row['projection']
                    if not evidence['outcome']['complete'] or evidence['source']['source_timestamp_s'] != frame['timestamp_s']:
                        reasons.append('incomplete projection or timestamp mismatch')
                elif row['kind'] == 'publication':
                    if frame['frame_id'] in publications:
                        reasons.append('duplicate publication outcome')
                    publications.add(frame['frame_id'])
                    if frame['publication'] not in ('successful','empty','rejected','abandoned'):
                        reasons.append('unknown publication outcome')
                    published_count = frame['published_poi_count']
                    if frame['publication'] == 'successful' and (type(published_count) is not int or published_count <= 0):
                        reasons.append('successful publication lacks positive POI count')
                    if frame['publication'] == 'empty' and published_count != 0:
                        reasons.append('empty publication has inconsistent POI count')
                else:
                    reasons.append('unknown record kind')
        if count != manifest['written'] or digest.hexdigest() != manifest['sha256']:
            reasons.append('persisted count or hash disagrees')
        if set(frames) != publications:
            reasons.append('frame lacks publication outcome')
        if expected_projections is not None and projections != expected_projections:
            reasons.append('expected projection count disagrees')
        return dict(valid=not reasons, reasons=sorted(set(reasons)), projections=projections,
                    records=count, frames=len(frames), manifest=manifest)
    except (OSError, ValueError, KeyError, TypeError) as error:
        return dict(valid=False, reasons=[f'{type(error).__name__}: {error}'])
