from yt_dlp import YoutubeDL
import os

# URL of the video
URL = "https://www.youtube.com/watch?v=_fybREErgyM"

# Folder to save the clip
output_folder = "datasets/videos/"
os.makedirs(output_folder, exist_ok=True)

# Full path + filename
output_file = os.path.join(output_folder, "my_clip_37-44.mp4")

ydl_opts = {
    "format": "18",  # 480p MP4
    "download_sections": ["*37-44"],  # clip from 37s to 44s
    "outtmpl": output_file,           # save to custom location/name
    "extractor_args": {
        "youtube": {"player_client": ["android"]}  # avoid 403 errors
    },
    "postprocessors": [{
        "key": "FFmpegVideoConvertor",
        "preferedformat": "mp4"
    }],
    "postprocessor_args": {
        "ffmpeg": ["-an"]  # remove audio track
    },
    "quiet": False,
}

with YoutubeDL(ydl_opts) as ydl:
    ydl.download([URL])
