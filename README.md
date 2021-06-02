# thumbs.py

Create thumbnail contact sheets for videos.

### Dependencies:

* python3.5+
* ffmpeg
* imagemagick

#

```
$ thumbs -h

usage: thumbs [-h] [-v] [-Q] [-o OUTPUT] [-q QUALITY] [-y] [-n] [-T]
              [-c TIMESTAMP_COLOR] [-b TIMESTAMP_BOX_COLOR]
              [-f TIMESTAMP_FONT] [-s TIMESTAMP_SIZE] [-w WATERMARK]
              [-B BACKGROUND] [-F FONT] [-S SIZE] [-C FONTCOLOR]
              [--vr {L,R,T,B}]
              width tiles video [video ...]

Create a video contact sheet

positional arguments:
  width                 Width in pixels of the output contact sheet.
  tiles                 How to tile contact sheet. Should be of the form "WxH"
                        (eg. "6x9" would result in a contact sheet that is 6
                        tiles wide and 9 tiles tall).
  video                 Path to the video file (or directory containing
                        videos) to create the contact sheet for.

optional arguments:
  -h, --help            show this help message and exit
  -v, --verbose         Emit debugging information. Specify multiple times to
                        increase verbosity (up to 3).
  -Q, --quiet           Silence all output except for errors. This overrides
                        -v flags.
  -o OUTPUT, --output OUTPUT
                        Path to the output contact sheet. If a directory is
                        specified, the output image will be saved there as
                        "VIDEO.thumbs.jpg". If the path does not exist, it is
                        assumed to be the destination if it ends with an
                        extension (eg. '.jpg') and the resulting image will be
                        saved there. If the path does NOT exist and does NOT
                        end with an extension, the directory will be created
                        and output images saved there. Defaults to
                        "VIDEO.thumbs.jpg" in the video's directory.
  -q QUALITY, --quality QUALITY
                        The contact sheet quality level: 1 (most compressed,
                        worst quality) -> 100 (least compressed, best
                        quality). Applies only to jpeg/miff/png.
  -y, --yes             Overwrite existing contact sheet image(s) without
                        asking.
  -n, --no              Skip generating a contact sheet if it already exists.
                        This overrides a -y flag (so specyfing both -y and -n
                        will cause the script to skip generating any existing
                        contact sheets.
  -T, --no-timestamp    Turn off timestamps. This overrides other
                        --timestamp-* options.
  -c TIMESTAMP_COLOR, --timestamp-color TIMESTAMP_COLOR
                        The fg color to use for timestamps. Should be of the
                        form "#RRGGBB[AA]".
  -b TIMESTAMP_BOX_COLOR, --timestamp-box-color TIMESTAMP_BOX_COLOR
                        The bg color to use for timestamps. Should be of the
                        form "#RRGGBB[AA]".
  -f TIMESTAMP_FONT, --timestamp-font TIMESTAMP_FONT
                        The name of the font to use for timestamps.
  -s TIMESTAMP_SIZE, --timestamp-size TIMESTAMP_SIZE
                        The font size of timestamps.
  -w WATERMARK, --watermark WATERMARK
                        Path to the watermark each screenshot with
  -B BACKGROUND, --background BACKGROUND
                        Background color of the contact sheet's empty space
  -F FONT, --font FONT  The name of the font to use for metadata.
  -S SIZE, --size SIZE  The font size of metadata.
  -C FONTCOLOR, --fontcolor FONTCOLOR
                        The font color of metdata.
  --vr {L,R,T,B}        Crop the thumbnails to a specific VR eye (Left, Right,
                                Top, Bottom half of the image)

```

