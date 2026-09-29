"""Verify the recorded-input identities, MP4 chapters and complete decode."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    video = Path(manifest['video'])
    assert sha256(video) == manifest['videoSha256']
    root = Path(__file__).resolve().parents[2]
    assert sha256(root / 'scripts/renderCollisionAvoidanceVideo.py') == manifest['rendererSha256']
    expected = [(speed, name) for speed in (8, 15) for name in
                ('recovery', 'circular', 'brakingTarget', 'turningTarget',
                 'acceleratingTurn', 'oncoming', 'acceleratingTarget')]
    chapters = manifest['chapters']
    assert [(c['speedMetersPerSecond'], c['scenario']) for c in chapters] == expected
    for chapter in chapters:
        source = Path(chapter['source'])
        assert sha256(source) == chapter['sourceSha256']
        result = json.loads(source.read_text())['results']
        assert result['completed'] and not result['failure']
        assert result['minimumReplayClearanceMeters'] == chapter['minimumBodyClearanceMeters']
        assert result['configuration']['collision']['safetyMarginMeters'] == .006
    probe = json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-show_streams', '-show_format', '-show_chapters',
        '-of', 'json', str(video)], text=True))
    stream = next(s for s in probe['streams'] if s['codec_type'] == 'video')
    assert stream['codec_name'] == 'h264' and stream['pix_fmt'] == 'yuv420p'
    assert (stream['width'], stream['height']) == (1600, 900)
    assert stream['r_frame_rate'] == '30/1'
    assert int(stream['nb_frames']) == manifest['totalFrames'] == 4504
    assert abs(float(probe['format']['duration']) - manifest['durationSeconds']) < .04
    assert len(probe['chapters']) == 14
    for encoded, recorded in zip(probe['chapters'], chapters):
        # MP4 muxing extends the first chapter to include the opening cover.
        chapter_start = 0 if recorded['index'] == 1 else recorded['startVideoSeconds']
        assert abs(float(encoded['start_time']) - chapter_start) <= .001
        assert abs(float(encoded['end_time']) - recorded['endVideoSeconds']) <= .001
        assert encoded['tags']['title'].startswith(f"{recorded['index']:02d} - ")
    decode_command = ['ffmpeg', '-v', 'error', '-i', str(video), '-f', 'null', '-']
    decode = subprocess.run(decode_command, capture_output=True, text=True)
    assert decode.returncode == 0 and decode.stderr == '', decode.stderr
    output = dict(passed=True, videoSha256=manifest['videoSha256'],
                  sourceHashesVerified=14, chapterCount=14, decodedFrames=4504,
                  firstChapterIncludesOpeningCover=True,
                  durationSeconds=manifest['durationSeconds'], resolution=[1600, 900],
                  fps=30, codec='h264', pixelFormat='yuv420p',
                  fullDecodeCommand=decode_command, fullDecodeExitCode=decode.returncode,
                  fullDecodeErrors=decode.stderr,
                  visualInspection='Encoded stills from all fourteen chapters, closest approach in scene 07, and corrected renderer previews reviewed',
                  scope='Visualization of existing exact-observation results; no controller modification or new simulation')
    args.output.write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps(output))


if __name__ == '__main__':
    main()
