#!/usr/bin/env python3

# ######################################################################
# Dependencies:
# - ffmpeg
# - imagemagick
# - python3.5+
# ######################################################################
#
# thumbs.py
#
# Create a video contact sheet
#
# Commands taken from:
# https://reddit.com/emjww6
# https://video.stackexchange.com/a/21564
#
# Other references used:
# https://ffmpeg.org/ffprobe.html
# https://ffmpeg.org/ffmpeg.html
# https://ffmpeg.org/ffmpeg-filters.html
# https://imagemagick.org/Usage/annotating/
# http://www.imagemagick.org/script/command-line-options.php
#
# ######################################################################

import argparse
from pathlib import Path
from subprocess import CompletedProcess
import sys
import time
from typing import (
        Any,
        List,
        Optional,
        Union,
)


VERBOSE: int = 0
ERR: int = -1

COLLAGE_X_OFFS = 3
COLLAGE_Y_OFFS = 3

def log(level: int, fmt: str,
        *args: Any, print_kwargs: dict=None, **kw: Any) -> None:
    """
    log emits a message if the VERBOSE level exceeds the one specified.
    """
    if VERBOSE < level:
        return

    if not isinstance(print_kwargs, dict):
        print_kwargs = {}
    if level == ERR:
        from sys import stderr
        print_kwargs.setdefault('file', stderr)
    if isinstance(fmt, Exception):
        fmt = '{0}: {1}'.format(fmt.__class__.__name__, fmt)
    elif isinstance(fmt, bytes):
        fmt = fmt.decode()

    if not isinstance(fmt, str):
        fmt = str(fmt)

    print(fmt.format(*args, **kw), **print_kwargs)

def runcmd(raw_cmd: str, *, stdin=None) -> CompletedProcess:
    import shlex
    from subprocess import (
            run,
            PIPE,
    )

    log(3, '$ {0}', raw_cmd, print_kwargs={'end': '\n\n'})

    cmd_split = shlex.split(raw_cmd)
    result = run(cmd_split, stdin=stdin, stdout=PIPE, stderr=PIPE)

    err = result.stderr
    if err:
        from math import (
                ceil,
                floor
        )

        err = err.strip()
        if isinstance(err, bytes):
            err = err.decode()
        width = 72
        wmsg = ' ERROR '
        left = int(width/2) - ceil(len(wmsg)/2)
        right = int(width/2) - floor(len(wmsg)/2)
        wrapper = '{0}{2}{1}'.format('!'*left, '!'*right, wmsg)
        log(ERR, wrapper)
        log(ERR, '$ {0}', raw_cmd, print_kwargs={'end': '\n\n'})
        log(ERR, '{0}', err)
        log(ERR, wrapper, end='\n\n')

    return result

class MediaInfo:
    def __init__(self, ffprobe_stdout: bytes, delim: str='|'):
        if not ffprobe_stdout:
            return

        if isinstance(ffprobe_stdout, bytes):
            ffprobe_stdout = ffprobe_stdout.decode()
        data = {}
        for line in [s.strip() for s in ffprobe_stdout.split('\n')]:
            parts = line.split('|')
            line_data = {}
            data_key = parts[0]

            for p in parts[1:]:
                try:
                    k, v = p.split('=')
                except ValueError:
                    continue
                if k == 'codec_type':
                    data_key = v
                line_data[k] = v

            # Only cache the first seen stream of each type (video, audio).
            data.setdefault(data_key, line_data)

        self.filename = Path(data['format']['filename']).name
        self.duration_sec = float(data['format']['duration'])
        self.size_bytes = int(data['format']['size'])
        self.bitrate = int(data['format']['bit_rate'])

        self.video_codec = data['video']['codec_long_name']
        self.video_profile = data['video']['profile']
        self.video_width = data['video']['width']
        self.video_height = data['video']['height']
        self.video_fps_raw = data['video']['r_frame_rate']
        try:
            self.video_bitdepth = int(data['video']['bits_per_raw_sample'])
        except ValueError:
            self.video_bitdepth = None

        self.has_audio = False
        if 'audio' in data:
            self.audio_codec = data['audio']['codec_long_name']
            self.audio_profile = data['audio']['profile']
            self.audio_channels = float(data['audio']['channels'])
            self.audio_chan_layout = data['audio']['channel_layout']
            self.audio_sample_rate = int(data['audio']['sample_rate'])
            try:
                self.audio_bitdepth = int(data['audio']['bits_per_raw_sample'])
            except ValueError:
                self.audio_bitdepth = None

            self.has_audio = True

        log(2, self, print_kwargs={'end': '\n\n'})

    def __str__(self) -> str:
        sep = '-'*36
        info = ['{0}:'.format(self.filename),
                'Duration: {0} ({1} seconds)'.format(
                    self.duration_readable, self.duration_sec),
                'Size: {0} ({1} bytes)'.format(
                    self.size_readable, self.size_bytes),
                'Bitrate: {0} ({1} bps)'.format(
                    self.bitrate_readable, self.bitrate),
                sep,
        ]

        has_video = False
        if self.video_codec:
            info.append('Video codec: {0}'.format(self.video_codec))
            has_video = True
        if self.video_profile:
            info.append('Video profile: {0}'.format(self.video_profile))
            has_video = True
        if self.video_width and self.video_height:
            info.append('Video resolution: {0} x {1}'.format(
                self.video_width, self.video_height))
            has_video = True
        if self.video_fps_raw:
            info.append('Video fps: {0} ({1})'.format(
                self.fps, self.video_fps_raw))
            has_video = True
        if has_video:
            info.append(sep)

        if self.has_audio:
            if self.audio_codec:
                info.append('Audio codec: {0}'.format(self.audio_codec))
            if self.audio_profile:
                info.append('Audio profile: {0}'.format(self.audio_profile))
            if self.audio_channels:
                info.append('Audio channels: {0}'.format(self.audio_channels))
            if self.audio_chan_layout:
                info.append('Audio channel layout: {0}'.format(self.audio_chan_layout))
            if self.audio_sample_rate:
                info.append('Audio sample rate: {0} Hz'.format(self.audio_sample_rate))
            if self.audio_bitdepth:
                info.append('Audio bitdepth: {0}'.format(self.audio_bitdepth))

        return '\n    '.join(info)

    @property
    def duration_readable(self) -> str:
        return time.strftime('%H:%M:%S', time.gmtime(self.duration_sec))

    @property
    def size_readable(self) -> str:
        # Human-readable size
        # https://stackoverflow.com/a/1094933
        def sizeof_fmt(num, suffix='B'):
            for unit in ['','Ki','Mi','Gi','Ti','Pi','Ei','Zi']:
                if abs(num) < 1024.0:
                    return "%3.02f %s%s" % (num, unit, suffix)
                num /= 1024.0
            return "%.02f %s%s" % (num, 'Yi', suffix)
        return sizeof_fmt(self.size_bytes)

    @property
    def bitrate_readable(self) -> str:
        def readable_bitrate(bitrate: int) -> str:
            for unit in ['', 'K', 'M', 'G', 'T', 'P', 'E', 'Z']:
                if abs(bitrate) < 1000.0:
                    return "%3.02f %sbps" % (bitrate, unit)
                bitrate /= 1000.0
            return "%.02f Ybps" % bitrate
        return readable_bitrate(self.bitrate)

    @property
    def sample_rate_readable(self) -> str:
        def readable_sample_rate(sr: int) -> str:
            for unit in ['', 'k', 'M', 'G', 'T', 'P', 'E', 'Z']:
                if abs(sr) < 1000.0:
                    return "%3.01f %sHz" % (sr, unit)
                sr /= 1000.0
            return "%.01f YHz" % sr
        return readable_sample_rate(self.audio_sample_rate)

    @property
    def resolution(self) -> str:
        return '{0} x {1}'.format(self.video_width, self.video_height)

    @property
    def fps(self) -> float:
        fps = -1
        parts = self.video_fps_raw.split('/')
        try:
            if len(parts) == 1:
                fps = float(parts[0])
            else:
                numerator = parts[0]
                denominator = parts[1]
                fps = float(numerator) / float(denominator)

        except (IndexError, ValueError):
            pass
        return fps

def media_info(path: Path) -> Optional[MediaInfo]:
    """
    media_info extracts media info about the specified video.

    Returns:
        MediaInfo: Media info about the video.
        None: If the info could not be extracted.
    """
    result = runcmd('ffprobe -loglevel error'
            ' -show_entries'
            ' stream=codec_long_name,profile,codec_type,width,height,'
            'r_frame_rate,channels,channel_layout,bits_per_raw_sample,'
            'sample_rate'
            ':format=filename,duration,size,bit_rate'
            ' -print_format "csv=item_sep=|:nokey=0"'
            ' "{0}"'.format(path))
    if result.returncode != 0:
        return None
    return MediaInfo(result.stdout)

def screenshot(path: Path, out: Path, seek: float, width: int, *,
        crop: str=None, ts: bool=True, ts_font: str=None,
        ts_color: str='#FFFFFFFF', ts_box_color: str='#00000080',
        ts_size: int=16, watermark: Path=None) -> Optional[Path]:
    """
    screenshot takes a single screenshot of the video.

    Arguments:
        path: The path to the video to take a screenshot of.
        out: The path to write the screenshot to.
        seek: Time in seconds to take a screenshot at.
        width: Output width of the screenshot.
        crop: How to crop the image, if any. Should be one of 'L', 'R',
                'T', or 'B'.
        ts: Whether to draw the timestamp of the video in the
            screenshot. This overrides the ts_* args if False.
        ts_font: The font to use for the timestamp.
        ts_color: The color to draw the timestamp.
        ts_box_color: The color of the timestamp box.
        ts_size: The font size to draw the timestamp.
        watermark: Path to the image to watermark the screenshot with.

    Returns:
        pathlib.Path: The path to the screenshot if it is successfully written.
        None: If the screenshot fails for any reason.
    """
    # Determine which filters to use, if any.
    filters = []

    if crop:
        if crop == 'L':
            filters.append('crop=iw/2:ih:0:0')
        elif crop == 'R':
            filters.append('crop=iw/2:ih:iw/2:0')
        elif crop == 'T':
            filters.append('crop=iw:ih/2:0:0')
        elif crop == 'B':
            filters.append('crop=iw:ih/2:0:ih/2')

        else:
            # This will be spammy.
            # TODO? Emit once
            log(ERR, '[WARNING] Unrecognized screenshot crop argument \'{0}\'.'
                    ' _NOT_ cropping screenshot.', crop)

    # Apply scale after cropping so that the image is scaled correctly.
    filters.append('scale={w}:-1'.format(w=width))

    # FIXME? Do -start_at_zero -copyts slow down processing? If so, should--
    # FIXME? -- remove if ts==False.
    result = runcmd('ffmpeg -start_at_zero -copyts' # Absolute timestamp values
            ' -hide_banner -loglevel error'
            ' -y' # Overwrite output files without asking
            ' -ss {seek}' # Screenshot at this time in the video
            ' -i "{path}"' # The input video
            ' -vframes:v 1' # Output a single frame of the video
            ' -vf "{filters}"'
            ' "{out}"'.format(seek=seek, path=path, filters=','.join(filters),
                out=out))
    if result.returncode != 0:
        return None

    if ts:
        from tempfile import TemporaryFile

        # Draw the timestamp on the image.
        # https://imagemagick.org/Usage/annotating/#anno_on
        stamp = time.strftime('%H:%M:%S', time.gmtime(seek))
        font = '-font {0}'.format(ts_font) if ts_font else ''
        size = '-pointsize {0}'.format(ts_size) if ts_size > 0 else ''
        with TemporaryFile() as tmp:
            pipe = runcmd('convert {font} {size}'
                    ' -fill \'{color}\''
                    ' -background \'{box}\''
                    ' label:\'{ts}\''
                    ' miff:-'.format(font=font, size=size,
                        color=ts_color, box=ts_box_color, ts=stamp))
            if pipe.returncode != 0:
                return None

            tmp.write(pipe.stdout)
            tmp.seek(0)

            result = runcmd('composite -gravity southeast'
                    ' -geometry +5+5'
                    ' - "{out}" "{out}"'.format(out=out), stdin=tmp)
            if result.returncode != 0:
                return None

    if watermark:
        result = runcmd('composite'
                # Resize the watermark to fit the screenshot
                # '{w}x\>'
                #   {w} => dynamically resize based on the screenshot width
                #   x   => width/height delim
                #   \>  => only shrink if the watermark is larger
                # See: http://www.imagemagick.org/script/command-line-processing.php#geometry
                ' \( "{watermark}" -resize {w}x\> \)'
                #' \( "{watermark}" -resize x{w} \)'
                ' -gravity center' # Center the watermark
                ' -watermark 10%' # Fade the watermark
                # Apply the watermark to the screenshot and overwrite it
                ' "{out}" "{out}"'.format(
                    w=int(width*0.90), watermark=watermark, out=out))
        if result.returncode != 0:
            return None

    if result.returncode == 0:
        return out
    return None

def stitch(width: int, height: int, quality: int, info: MediaInfo,
        screens: List[Path], out: Path, *,
        background: str='#232323',
        font: str=None, size: int=18, color: str='#FFFFFFFF') -> Optional[Path]:
    """
    stitch joins the input screenshots together into a single image of rows-
        cells high and cols-cells wide.

    Arguments:
        width: The number of tiles wide of the output image.
        height: The number of tiles tall of the output image.
        info: MediaInfo used to add video metadata to the contact sheet.
        screens: List of input screenshots to join into the output image.
        out: The path to write the output image to.

        background: The background color of any empty space in the output
                    image.
        font: The font to use to draw metadata.
        size: The font point size to use to draw metadata.
        color: The metadata font color.

    Returns:
        pathlib.Path: The output image's path if successful.
        None: If stitch() fails for any reason.
    """
    out.parent.mkdir(mode=0o755, parents=True, exist_ok=True)

    result = runcmd('montage -mode concatenate' # Concat the images together
            ' -quality {q}' # Set compression level of the contact sheet
            ' -tile {w}x{h}' # Arrange the contact sheet
            ' -geometry +{xoffs}+{yoffs}' # Space out the images
            ' -background {background}' # Set the background color
            ' {infiles} "{outfile}"'.format(w=width, h=height, q=quality,
                xoffs=COLLAGE_X_OFFS, yoffs=COLLAGE_Y_OFFS,
                background=background,
                infiles=' '.join(['"{0}"'.format(s) for s in screens]),
                outfile=out))
    if result.returncode != 0:
        return None

    # Format the media info.
    data = [
            '{filename}'.format(filename=info.filename),

            '  Size: {size} / Duration: {duration}'
            ' / Bitrate: {bitrate}'.format(
                size=info.size_readable, duration=info.duration_readable,
                bitrate=info.bitrate_readable),
    ]

    data.append(
        '  Video: {vc} ({vprof})'.format(
            vc=info.video_codec, vprof=info.video_profile),
    )
    if isinstance(info.video_bitdepth, int):
        data[-1] += ' / Bit depth: {0}'.format(info.video_bitdepth)
    data[-1] += ('\\n       '
            'Resolution: {res} @ {fps:.1f} fps ({fps_raw})'.format(
                res=info.resolution, fps=info.fps, fps_raw=info.video_fps_raw))

    if info.has_audio:
        data.append(
                '  Audio: {ac} ({aprof}) / {chan} Channel{s} ({layout})'
                ' / Sample rate: {sr}'.format(
                ac=info.audio_codec, aprof=info.audio_profile,
                s='s' if info.audio_channels != 1 else '',
                chan=info.audio_channels, layout=info.audio_chan_layout,
                sr=info.sample_rate_readable),
        )
        if isinstance(info.audio_bitdepth, int):
            data[-1] += ' / Bit depth: {0}'.format(info.audio_bitdepth)

    # Draw the media info.
    font = '-font {0}'.format(font) if font else ''
    size = '-pointsize {0}'.format(size) if size > 0 else ''
    color = '-fill {0}'.format(color) if color else ''
    result = runcmd('convert "{outfile}"' # Take the contact sheet as input
            ' -background {background}' # Set the background color
            ' {font} {size} {color}' # Specify font settings
            ' label:\'{data}\'' # Specify the text to draw
            ' +swap' # Place the data above the contact sheet
            ' -append' # Join the images together
            ' "{outfile}"'.format( # Overwrite the contact sheet
                background=background,
                outfile=out, font=font, size=size, color=color,
                data='\\n'.join(data)))
    if result.returncode != 0:
        return None
    return out

def get_videos(path: Path) -> List[Path]:
    """
    get_videos recursively searches the specified path for video files.

    Arguments:
        path: The directory to search for video files.

    Returns:
        list: List of paths of all videos found, if any. If the given
                path is not a directory, a list with the given path is
                returned if the path is a video.
    """
    def search(path: Path, *, depth: int=0):
        result = []

        path = sanitize_path(path)
        log(1, '{pre}{name}{end}', pre='-'*depth*2,
                name=path.name or '.',
                end='/' if path.is_dir() else '')

        for i, child in enumerate(path.iterdir()):
            log(1, '{pre}{dot:<6}', pre=' '*(depth*2+1), dot='.'*((i%3)+1),
                    print_kwargs={'end': '\r'})

            if child.is_dir():
                result += search(child, depth=depth+1)

            else:
                child_ffprobe = runcmd('ffprobe -loglevel error'
                        ' -show_entries format=format_name'
                        ' -print_format "default=noprint_wrappers=1:nokey=1"'
                        ' -i "{0}"'.format(child))
                if child_ffprobe.returncode != 0:
                    continue
                format_name = child_ffprobe.stdout.strip()
                if isinstance(format_name, bytes):
                    format_name = format_name.decode()

                log(3, '\tformat_name: {fn}\n', child=child, fn=format_name)

                # Check if the file is a video.
                #
                # common video format_name:
                # mp4,mov  => mov,mp4,m4a,3gp,3g2,mj2
                # mkv,webm => matroska,webm
                # wmv      => asf
                # flv      => flv
                # avi      => avi
                # ts       => mpegts
                if format_name in ('asf', 'flv', 'avi', 'mpegts',
                        # TODO? Only partially match these in case they change?
                        'mov,mp4,m4a,3gp,3g2,mj2',
                        'matroska,webm'):
                    result.append(child)
        log(1, '{pre}Found: #{n}', pre=' '*(depth*2+1), n=len(result))
        return result

    wrapper = '-'*36
    path = sanitize_path(path)
    log(0, 'Searching \'{0}\' for videos ... ', path, print_kwargs={'end': ''})
    log(1, '\n' + wrapper)

    result = []
    try:
        result = search(path)
    except OSError:
        result.append(sanitize_path(path))
    finally:
        log(0, 'Search done!')
        log(1, wrapper)
        return result

def format_outpaths(video: List[Path], parent: Path=None,
        name: str='{0}.thumbs.jpg', default_ext='.jpg') -> List[Path]:
    from os.path import splitext

    paths = []

    # Flag to warn a single time if a static output path is defined for
    # multiple input paths.
    warned = False

    for i, p in enumerate(video):
        outname = name.format(p.stem)

        if len(video) > 1 and outname == name:
            # A static output name was specified for multiple input videos.
            if not warned:
                warned = True
                log(0, '\n[WARNING] A single output path given for'
                        ' #{n} input videos. Inserting an index to'
                        ' output image files.\n',
                        n=len(video))
            prefix, suffix = splitext(outname)
            if not suffix:
                suffix = default_ext
            # 'foo.06.jpg'
            outname = '{0}.{1:0{2}d}{3}'.format(
                    prefix, i+1, len(str(len(video))), suffix)

        if parent is None:
            # Place the output in the same directory as the video.
            out = p.with_name(outname)
        else:
            # Place the output in the specified directory.
            out = Path(parent, outname)
        # Ensure the output file has an extension (hopefully an image ext).
        if not out.suffix:
            out = out.with_suffix(default_ext)
        paths.append(sanitize_path(out))
    return paths

def init_parser() -> argparse.ArgumentParser:
    # This function exists to aid readability of parse_args()
    parser = argparse.ArgumentParser(
            description='Create a video contact sheet')

    parser.add_argument('-v', '--verbose', action='count',
            help='Emit debugging information. Specify multiple times to'
            ' increase verbosity (up to 3).')
    parser.add_argument('-Q', '--quiet', action='store_true',
            help='Silence all output except for errors. This overrides'
            ' -v flags.')

    parser.add_argument('-o', '--output',
            help='Path to the output contact sheet. If a directory is'
            ' specified, the output image will be saved there as'
            ' "VIDEO.thumbs.jpg". If the path does not exist, it is assumed'
            ' to be the destination if it ends with an extension '
            ' (eg. \'.jpg\') and the resulting image will be saved there.'
            ' If the path does NOT exist and does NOT end with an extension,'
            ' the directory will be created and output images saved there.'
            ' Defaults to "VIDEO.thumbs.jpg" in the video\'s directory.')
    parser.add_argument('-q', '--quality', type=int, default=92,
            help='The contact sheet quality level: '
            '1 (most compressed, worst quality) ->'
            ' 100 (least compressed, best quality).'
            ' Applies only to jpeg/miff/png.')

    parser.add_argument('-y', '--yes', action='store_true',
            help='Overwrite existing contact sheet image(s) without asking.')
    parser.add_argument('-n', '--no', action='store_true',
            help='Skip generating a contact sheet if it already exists.'
            ' This overrides a -y flag (so specyfing both -y and -n will cause'
            ' the script to skip generating any existing contact sheets.')

    parser.add_argument('-T', '--no-timestamp', action='store_true',
            help='Turn off timestamps. This overrides other --timestamp-*'
            ' options.')
    parser.add_argument('-c', '--timestamp-color', default='#FFFFFFFF',
            help='The fg color to use for timestamps. Should be of the form'
            ' "#RRGGBB[AA]".')
    parser.add_argument('-b', '--timestamp-box-color', default='#00000080',
            help='The bg color to use for timestamps. Should be of the form'
            ' "#RRGGBB[AA]".')
    parser.add_argument('-f', '--timestamp-font', default='Arial',
            help='The name of the font to use for timestamps.')
    parser.add_argument('-s', '--timestamp-size', default=16, type=int,
            help='The font size of timestamps.')

    parser.add_argument('-w', '--watermark',
            help='Path to the watermark each screenshot with')

    parser.add_argument('-B', '--background', default='#232323',
            help='Background color of the contact sheet\'s empty space')
    parser.add_argument('-F', '--font', default='Arial',
            help='The name of the font to use for metadata.')
    parser.add_argument('-S', '--size', default=20, type=int,
            help='The font size of metadata.')
    parser.add_argument('-C', '--fontcolor', default='#FFFFFFFF',
            help='The font color of metdata.')

    parser.add_argument('--vr', choices=('L', 'R', 'T', 'B'),
            help='Crop the thumbnails to a specific VR eye'
            ' (Left, Right, Top, Bottom half of the image)')

    parser.add_argument('width', type=int,
            help='Width in pixels of the output contact sheet.')
    parser.add_argument('tiles',
            help='How to tile contact sheet. Should be of the form'
            ' "WxH" (eg. "6x9" would result in a contact sheet that'
            ' is 6 tiles wide and 9 tiles tall).')
    parser.add_argument('video', nargs='+',
            help='Path to the video file (or directory containing videos) to'
            ' create the contact sheet for.')
    return parser

def parse_args() -> Union[argparse.Namespace, int]:
    """
    Returns:
        argparse.Namespace: The parsed cli arguments.
        int: The exit code if the script should exit.
    """
    parser = init_parser()
    args = parser.parse_args()

    # Set verbosity immediately so that log() calls work.
    global VERBOSE
    VERBOSE = args.verbose or 0
    if args.quiet:
        VERBOSE = -1

    # Parse tile width and height.
    tiles = args.tiles.lower().split('x')
    try:
        args.tiles_width = int(tiles[0])
        args.tiles_height = int(tiles[1])
    except ValueError:
        log(ERR, 'Invalid tiles argument \'{tiles}\'. Should be of the form'
                ' "WxH" where W and H are ints.', tiles=args.tiles)
        return 1

    # Ensure 0 < quality <= 100
    if args.quality <= 0 or 100 < args.quality:
        args.quality = 92

    if args.watermark:
        args.watermark = sanitize_path(args.watermark)
        log(0, 'Watermarking screenshots with \'{0}\'', args.watermark)

    # Search for videos in the specified path.
    video = []
    for v in args.video:
        orig_path = Path(v)
        video += get_videos(orig_path)
        if len(video) == 0:
            log(ERR, 'No videos found from \'{0}\'', orig_path)
    args.video = video
    if len(args.video) == 0:
        log(ERR, '\nNo videos found!')
        return 0

    # Assign appropriate outputs based on the video(s) found and the output
    # specified by the user.
    orig_out = None
    if args.output is None:
        # Assign outputs to the same directory as their corresponding input.
        args.output = format_outpaths(args.video)

    else:
        orig_out = Path(args.output)
        if orig_out.exists():
            if orig_out.is_dir():
                # Assign outputs to the specified directory.
                args.output = format_outpaths(args.video, orig_out)
            else:
                args.output = format_outpaths(args.video,
                        orig_out.parent, orig_out.name)

        else: # The specified output path does not exist.
            if orig_out.suffix:
                # The user-specified output path has an extension. Assume the
                # user wants to use this as the exact output name.
                args.output = format_outpaths(args.video,
                        orig_out.parent, orig_out.name)

            else:
                # The user-specified output path has no extension. Assume the
                # user wants to place output images in this directory.
                args.output = format_outpaths(args.video, orig_out)

    if VERBOSE >= 2:
        import os

        log(2, 'Video -> Output: {0}', '-'*10)
        for i, v in enumerate(args.video):
            pre = ''.format(os.sep)
            out = args.output[i]
            try:
                out = out.relative_to(v.parent)
            except ValueError:
                pass # Output directory is not relative to the input parent
            else:
                pre = '...' + os.sep

            log(2, '{input}\n\t-> {pre}{output}', input=v, output=out, pre=pre,
                    print_kwargs={'end': '\n\n'})
        log(2, '-'*72, print_kwargs={'end': '\n\n'})

    return args

def sanitize_path(path: Path) -> Path:
    # Fix paths if running python under cygwin.
    if sys.platform == 'cygwin':
        result = runcmd('cygpath --mixed "{0}"'.format(path))
        if result.returncode != 0:
            sys.exit(1)
        path = result.stdout.strip()
        if isinstance(path, bytes):
            path = path.decode()

    return Path(path)

HAS_DEPS = True

def check_dep(cmd: str, pkg: str):
    import shutil

    global HAS_DEPS

    if shutil.which(cmd) is None:
        HAS_DEPS = False
        log(ERR, 'Could not locate `{0}` (is "{1}" installed?)', cmd, pkg)

if __name__ == '__main__':
    # Emit a blank line immediately to help readability of the script's output.
    log(0, '')

    opts = parse_args()

    try:
        if isinstance(opts, int):
            sys.exit(opts)

        check_dep('ffprobe', 'ffmpeg')
        check_dep('ffmpeg', 'ffmpeg')
        check_dep('convert', 'imagemagick')
        check_dep('composite', 'imagemagick')
        check_dep('montage', 'imagemagick')
        if not HAS_DEPS:
            sys.exit(1)

        from tempfile import TemporaryDirectory

        with TemporaryDirectory() as tmpdir:
            tmpdir = sanitize_path(tmpdir)

            for i, video in enumerate(opts.video):
                log(0, '\n' + '='*72)
                log(0, 'Processing: {path}', path=video)
                log(0, '='*72, print_kwargs={'end': '\n\n'})

                info = media_info(video)
                if info is None:
                    sys.exit(1)

                num_screens = opts.tiles_width * opts.tiles_height
                # Trim off 5% of the duration so that ffmpeg doesn't hit the
                # end of the video and fail to create the last screenshot.
                interval = (0.95*info.duration_sec) / num_screens

                # Calculate the screenshot width.
                # COLLAGE_X_OFFS*2: each screenshot in the collage will have
                #                   COLLAGE_X_OFFS pixels on its left and right
                targetwidth = (opts.width - (COLLAGE_X_OFFS*2)*opts.tiles_width)
                ss_width = int(targetwidth / opts.tiles_width)

                output = opts.output[i]

                log(0, 'Saving contact sheet to: {out}',
                        out=output, print_kwargs={'end': '\n' + '-'*36 + '\n'})

                if output.exists():
                    log(0, 'Contact sheet already exists!')
                    if not (opts.yes or opts.no):
                        inp = None
                        while inp is None:
                            inp = input('Overwrite? [y/n] ').lower().strip()
                            if inp not in ('y', 'n'):
                                log(ERR, 'Unrecognized input: \'{0}\'', inp)
                                inp = None
                                continue

                            elif inp == 'n':
                                opts.no = True

                            elif inp == 'y':
                                opts.yes = True

                    if opts.no:
                        log(0, 'Skipping {out}', out=output,
                                print_kwargs={'end': '\n\n'})
                        continue
                    else:
                        log(0, '\n\tOverwriting \'{out}\'', out=output)

                log(0, 'Generating {n} screenshots (every {t:.1f} seconds) ...',
                        n=num_screens, t=interval)
                log(0, 'Contact sheet dimensions: {w} x {h}',
                        w=opts.tiles_width, h=opts.tiles_height,
                        print_kwargs={'end': '\n\n'})

                screens = []
                for i in range(num_screens):
                    log(0, 'Generating screenshot #{0:0{1}d} ...',
                            i+1, len(str(num_screens)),
                            print_kwargs={'end': '\r'})

                    tmpout = Path(tmpdir,
                            '{0}_{1:03d}.png'.format(video.stem, i))
                    path = screenshot(video, tmpout, interval*(i+1), ss_width,
                            crop=opts.vr,
                            ts=not opts.no_timestamp,
                            ts_font=opts.timestamp_font,
                            ts_color=opts.timestamp_color,
                            ts_box_color=opts.timestamp_box_color,
                            ts_size=opts.timestamp_size,
                            watermark=opts.watermark)
                    if path is None:
                        sys.exit(1)
                    screens.append(path)

                log(0, '\n\tDone!')

                # NOTE: If the output path did not exist at the initial check
                # but does now, then this will overwrite the file regardless of
                # the user's decision.
                if output.exists():
                    log(1, 'Overwriting \'{out}\' now', out=output)

                log(0, 'Creating contact sheet ... ', print_kwargs={'end': ''})
                stitch(opts.tiles_width, opts.tiles_height,
                        opts.quality, info, screens, output,
                        background=opts.background,
                        font=opts.font, size=opts.size, color=opts.fontcolor)
                log(0, '\tDone!')

    finally:
        log(0, 'Exiting ...')

