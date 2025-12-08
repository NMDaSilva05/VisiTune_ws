import os
import shutil
import re
import time

from pathlib import Path
import csv
import yt_dlp
from tqdm import tqdm

from PIL import Image
from io import BytesIO

# === CONFIG ===
# Folder where your existing songs live
MUSIC_SRC_DIR = Path("/Users/aidanbaker/Desktop/MusicForSTM32")
# Folder where your cover images live (can be the same as MUSIC_SRC_DIR)
IMAGE_SRC_DIR = Path("/Users/aidanbaker/Desktop/MusicForSTM32")
# Destination for your jukebox input
INPUT_DIR = Path("input")

# Path to your default cover image
DEFAULT_COVER = IMAGE_SRC_DIR / "default_cover.png"   # make sure this exists

# YouTube download settings
YOUTUBE_URLS_FILE = None  # Set to a file path if you want to batch download from URLs
# Example: YOUTUBE_URLS_FILE = "youtube_links.txt"
YOUTUBE_DOWNLOAD_DIR = MUSIC_SRC_DIR
YOUTUBE_DOWNLOAD_DIR.mkdir(exist_ok=True)

# Audio format settings
AUDIO_FORMAT = "mp3"  # or "m4a", "wav", "opus", "flac"
AUDIO_QUALITY = "320"  # 0 (best) to 9 (worst) OR "128", "192", "256", "320" for mp3

SUPPORTED_AUDIO = [".mp3", ".wav", ".aac", ".flac", ".ogg", ".m4a"]
SUPPORTED_IMAGES = [".jpg", ".jpeg", ".png", ".bmp", ".gif", ".webp", ".tiff"]


class YouTubeDownloader:
    """Class to handle YouTube downloads using yt-dlp"""
    
    def __init__(self):
        # Base options
        self.base_opts = {
            'format': 'bestaudio/best',
            'quiet': False,
            'no_warnings': False,
            'ignoreerrors': True,
            'nooverwrites': True,
            'continuedl': True,
            'writethumbnail': True,
            'prefer_ffmpeg': True,
        }
        
        # Track download progress
        self.current_download = None
    
    def create_ydl_opts(self, output_dir, single_video=True):
        """Create yt-dlp options based on download type"""
        opts = self.base_opts.copy()
        
        # Output template
        if single_video:
            opts['outtmpl'] = str(output_dir / '%(title)s.%(ext)s')
            opts['noplaylist'] = True  # Force single video even if URL has playlist
        else:
            opts['outtmpl'] = str(output_dir / '%(playlist)s' / '%(title)s.%(ext)s')
            opts['noplaylist'] = False
        
        # Audio extraction
        opts['postprocessors'] = [{
            'key': 'FFmpegExtractAudio',
            'preferredcodec': AUDIO_FORMAT,
            'preferredquality': str(AUDIO_QUALITY),
        }]
        
        # Metadata and thumbnail
        opts['embedthumbnail'] = True
        opts['postprocessor_args'] = [
            '-metadata', 'title=%(title)s',
            '-metadata', 'artist=%(uploader)s',
        ]
        
        # Progress hooks
        opts['progress_hooks'] = [self.progress_hook]
        
        return opts
    
    def progress_hook(self, d):
        """Hook to track download progress for tqdm"""
        if self.current_download:
            if d['status'] == 'downloading':
                if 'total_bytes' in d:
                    self.current_download['total'] = d['total_bytes']
                if 'downloaded_bytes' in d:
                    downloaded = d['downloaded_bytes']
                    if 'last_update' in self.current_download:
                        increment = downloaded - self.current_download['last_update']
                        self.current_download['pbar'].update(increment)
                    self.current_download['last_update'] = downloaded
            elif d['status'] == 'finished':
                if 'pbar' in self.current_download:
                    self.current_download['pbar'].close()
    
    def is_playlist_url(self, url):
        """Check if URL is a playlist"""
        playlist_indicators = ['list=', 'playlist', '&list=', '/playlist', 'mix', 'watch?v=']
        return any(indicator in url.lower() for indicator in playlist_indicators)
    
    def get_video_info(self, url, single_video=True):
        """Get video information using yt-dlp"""
        try:
            opts = {'quiet': True, 'no_warnings': True}
            if single_video:
                opts['noplaylist'] = True
            
            with yt_dlp.YoutubeDL(opts) as ydl:
                info = ydl.extract_info(url, download=False)
                
                # Check if it's actually a playlist
                if not single_video and '_type' in info and info['_type'] == 'playlist':
                    return info
                elif single_video and '_type' in info and info['_type'] == 'playlist':
                    # User asked for single but got playlist - extract first video
                    if 'entries' in info and info['entries']:
                        return info['entries'][0]
                
                return info
        except Exception as e:
            print(f"Error getting video info: {e}")
            return None
    
    def download_single_video(self, youtube_url, output_dir=None, quality=None):
        """
        Download a SINGLE YouTube video as audio
        
        Returns:
            Path to downloaded file or None if failed
        """
        if output_dir is None:
            output_dir = YOUTUBE_DOWNLOAD_DIR
        
        # Create options for single video
        ydl_opts = self.create_ydl_opts(output_dir, single_video=True)
        
        # Update quality if specified
        if quality:
            if AUDIO_FORMAT == 'mp3':
                ydl_opts['postprocessors'][0]['preferredquality'] = str(quality)
            else:
                ydl_opts['postprocessors'][0]['preferredquality'] = str(quality)
        
        print(f"Downloading single video: {youtube_url}")
        
        try:
            # Create progress bar
            with tqdm(
                desc="Downloading",
                unit='B',
                unit_scale=True,
                unit_divisor=1024,
                miniters=1,
                leave=True
            ) as pbar:
                # Initialize progress tracking
                self.current_download = {
                    'pbar': pbar,
                    'total': None,
                    'downloaded': 0,
                    'last_update': 0
                }
                
                # Download with yt-dlp
                with yt_dlp.YoutubeDL(ydl_opts) as ydl:
                    info = ydl.extract_info(youtube_url, download=True)
                    
                    if not info:
                        print(f"Failed to download: {youtube_url}")
                        return None
                    
                    # Get the downloaded file path
                    title = info.get('title', 'unknown')
                    # Sanitize filename
                    safe_title = re.sub(r'[<>:"/\\|?*]', '', title)
                    safe_title = re.sub(r'[^\w\s\-_\.]', '', safe_title)
                    safe_title = safe_title.replace(' ', '_')[:100]
                    
                    # Find the downloaded file
                    expected_filename = f"{safe_title}.{AUDIO_FORMAT}"
                    filepath = output_dir / expected_filename
                    
                    # Check for actual file
                    if not filepath.exists():
                        # Look for any audio file with similar name
                        for ext in [f'.{AUDIO_FORMAT}', '.m4a', '.webm', '.opus']:
                            for file in output_dir.glob(f"*{ext}"):
                                if safe_title[:50] in file.stem or 'youtube' in file.stem:
                                    filepath = file
                                    break
                    
                    print(f"✓ Downloaded: {title}")
                    return filepath
        
        except Exception as e:
            print(f"Download failed: {e}")
            return None
        finally:
            self.current_download = None
    
    def download_playlist(self, playlist_url, output_dir=None, max_items=None):
        """Download YouTube playlist with confirmation"""
        if output_dir is None:
            output_dir = YOUTUBE_DOWNLOAD_DIR
        
        # Get playlist info first
        print("Getting playlist information...")
        playlist_info = self.get_video_info(playlist_url, single_video=False)
        
        if not playlist_info or '_type' not in playlist_info or playlist_info['_type'] != 'playlist':
            print("Not a valid playlist. Trying as single video...")
            return self.download_single_video(playlist_url, output_dir)
        
        playlist_title = playlist_info.get('title', 'Unknown Playlist')
        playlist_count = len(playlist_info.get('entries', []))
        
        print(f"\nPlaylist: {playlist_title}")
        print(f"Number of videos: {playlist_count}")
        
        if max_items and playlist_count > max_items:
            print(f"Limiting to {max_items} videos")
            playlist_count = max_items
        
        # Ask for confirmation
        confirm = input(f"\nDownload {playlist_count} videos? (y/n): ").strip().lower()
        if confirm != 'y':
            print("Cancelled.")
            return []
        
        # Create options for playlist
        ydl_opts = self.create_ydl_opts(output_dir, single_video=False)
        ydl_opts['playlistend'] = max_items if max_items else playlist_count
        
        try:
            with yt_dlp.YoutubeDL(ydl_opts) as ydl:
                info = ydl.extract_info(playlist_url, download=True)
                
                downloaded_files = []
                if info and 'entries' in info:
                    for entry in info['entries']:
                        if entry and 'title' in entry:
                            print(f"✓ Downloaded: {entry['title']}")
                            # Find the file
                            for ext in [f'.{AUDIO_FORMAT}', '.m4a', '.webm', '.opus']:
                                for file in output_dir.glob(f"**/*{ext}"):
                                    if entry['title'][:50].replace(' ', '_') in file.stem:
                                        downloaded_files.append(file)
                                        break
                
                print(f"\n✓ Downloaded {len(downloaded_files)} videos from playlist")
                return downloaded_files
                
        except Exception as e:
            print(f"Playlist download failed: {e}")
            return []
    
    def download_thumbnail(self, video_id, output_dir, title=None):
        """Download YouTube thumbnail and center-crop to a square."""
        if not video_id:
            return None
        
        thumbnail_urls = [
            f'https://img.youtube.com/vi/{video_id}/maxresdefault.jpg',
            f'https://img.youtube.com/vi/{video_id}/sddefault.jpg',
            f'https://img.youtube.com/vi/{video_id}/hqdefault.jpg',
            f'https://img.youtube.com/vi/{video_id}/mqdefault.jpg',
            f'https://img.youtube.com/vi/{video_id}/default.jpg'
        ]
        
        safe_title = re.sub(r'[<>:"/\\|?*]', '', title) if title else video_id
        safe_title = safe_title.replace(' ', '_')[:50]
        
        import requests
        for url in thumbnail_urls:
            try:
                response = requests.get(url, timeout=10)
                if response.status_code != 200:
                    continue

                # Open image from bytes
                img = Image.open(BytesIO(response.content)).convert("RGB")
                w, h = img.size

                # Center square crop
                side = min(w, h)
                left = (w - side) // 2
                top = (h - side) // 2
                right = left + side
                bottom = top + side
                img_cropped = img.crop((left, top, right, bottom))

                # Save cropped thumbnail
                thumbnail_path = output_dir / f"{safe_title}_{video_id}.jpg"
                img_cropped.save(thumbnail_path, format="JPEG", quality=95)

                return thumbnail_path
            except Exception as e:
                # Try next resolution
                continue
        
        return None



def download_youtube_playlist(file_path, max_items=None):
    """Batch download YouTube videos from a text file"""
    downloader = YouTubeDownloader()
    
    if not os.path.exists(file_path):
        print(f"URL file not found: {file_path}")
        return []
    
    with open(file_path, 'r') as f:
        urls = [line.strip() for line in f if line.strip() and not line.startswith('#')]
    
    downloaded_files = []
    for i, url in enumerate(urls, 1):
        print(f"\n[{i}/{len(urls)}] Processing: {url}")
        
        if downloader.is_playlist_url(url):
            result = downloader.download_playlist(url, max_items=max_items)
            if result:
                downloaded_files.extend(result)
        else:
            result = downloader.download_single_video(url)
            if result:
                downloaded_files.append(result)
        
        # Add small delay to avoid rate limiting
        time.sleep(0.5)
    
    return downloaded_files


def find_image_for_stem(stem: str):
    """
    Look in IMAGE_SRC_DIR for an image with the same stem.
    Return Path or None.
    """
    for ext in SUPPORTED_IMAGES:
        candidate = IMAGE_SRC_DIR / f"{stem}{ext}"
        if candidate.is_file():
            return candidate
        
        # Also check for files containing the stem
        for file in IMAGE_SRC_DIR.glob(f"*{stem}*{ext}"):
            if file.is_file():
                return file
    
    return None


def parse_title_artist_from_filename(fname: str, video_info=None):
    """
    Try to parse 'Artist - Title.ext' -> (title, artist).
    For YouTube downloads, try to extract from filename or use video_info.
    """
    stem = Path(fname).stem
    
    # If we have video info from yt-dlp, use it
    if video_info:
        title = video_info.get('title', stem)
        artist = video_info.get('uploader', 'YouTube')
        channel = video_info.get('channel', '')
        
        # Try to extract artist from title
        if " - " in title:
            parts = title.split(" - ", 1)
            if len(parts) == 2:
                possible_artist, possible_title = parts[0].strip(), parts[1].strip()
                if len(possible_artist) < 50 and len(possible_title) < 100:
                    return possible_title, possible_artist
        
        return title, artist
    
    # Fallback: try to parse from filename
    if " - " in stem:
        clean_stem = re.sub(r'_[a-zA-Z0-9_-]{11}$', '', stem)
        if " - " in clean_stem:
            artist, title = clean_stem.split(" - ", 1)
            return title.strip(), artist.strip()
    
    return stem.strip(), "unknown"


def process_audio_file(audio_path, track_num, youtube_downloader=None):
    """Process a single audio file and create track folder"""
    stem = audio_path.stem
    image_path = find_image_for_stem(stem)
    
    # Check if this is a YouTube download
    video_id_match = re.search(r'([a-zA-Z0-9_-]{11})', stem)
    video_info = None
    
    if video_id_match and youtube_downloader:
        video_id = video_id_match.group(1)
        youtube_url = f"https://www.youtube.com/watch?v={video_id}"
        video_info = youtube_downloader.get_video_info(youtube_url)
        
        # Look for thumbnail with video ID
        if not image_path:
            for ext in SUPPORTED_IMAGES:
                for file in IMAGE_SRC_DIR.glob(f"*{video_id}*{ext}"):
                    if file.is_file():
                        image_path = file
                        print(f"Found thumbnail for {stem}")
                        break
    
    if image_path is None:
        if DEFAULT_COVER.is_file():
            print(f"[INFO] No matching image for '{audio_path.name}', using default cover.")
            image_path = DEFAULT_COVER
        else:
            print(f"[WARN] No matching image and no default cover for '{audio_path.name}', skipping.")
            return None
    
    # Create track folder
    track_dir = INPUT_DIR / f"track{track_num}"
    track_dir.mkdir(parents=True, exist_ok=True)
    
    # Copy audio + image
    dest_audio = track_dir / audio_path.name
    dest_image = track_dir / image_path.name
    
    shutil.copy2(audio_path, dest_audio)
    shutil.copy2(image_path, dest_image)
    
    # Build meta.csv
    title, artist = parse_title_artist_from_filename(audio_path.name, video_info)
    album = "YouTube" if video_info else "unknown"
    
    meta_path = track_dir / "meta.csv"
    with meta_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["title", "artist", "album"])
        writer.writerow([title, artist, album])
    
    print(f"[OK] Created {track_dir}")
    return track_dir


def main():
    # Step 1: Clear the input folder before starting
    print(f"\nClearing existing content in '{INPUT_DIR}'...")
    if INPUT_DIR.exists():
        for item in INPUT_DIR.iterdir():
            if item.is_dir():
                shutil.rmtree(item)
            else:
                item.unlink()
        print(f"✓ Cleared {INPUT_DIR}")
    INPUT_DIR.mkdir(exist_ok=True)
    
    # Initialize YouTube downloader
    youtube_downloader = YouTubeDownloader()
    
    # Step 2: Download from YouTube if URL file is specified
    downloaded_files = []
    if YOUTUBE_URLS_FILE:
        print(f"\nDownloading from {YOUTUBE_URLS_FILE}...")
        # Ask for max items if it's a playlist
        max_items = None
        if input("Limit playlist downloads? (y/n): ").strip().lower() == 'y':
            try:
                max_items = int(input("Max number of videos: "))
            except:
                pass
        
        downloaded_files = download_youtube_playlist(YOUTUBE_URLS_FILE, max_items=max_items)
        print(f"Downloaded {len(downloaded_files)} files")
    
    # Step 3: Collect all audio files
    audio_files = []
    
    # Add existing audio files
    for path in MUSIC_SRC_DIR.glob("*"):
        if path.suffix.lower() in SUPPORTED_AUDIO and path.is_file():
            audio_files.append(path)
    
    # Add downloaded YouTube files
    for path in YOUTUBE_DOWNLOAD_DIR.glob(f"*.{AUDIO_FORMAT}"):
        audio_files.append(path)
    
    # Look in subdirectories too (for playlists)
    for path in YOUTUBE_DOWNLOAD_DIR.rglob(f"*.{AUDIO_FORMAT}"):
        audio_files.append(path)
    
    # Remove duplicates
    audio_files = list(set(audio_files))
    audio_files.sort()
    
    if not audio_files:
        print(f"\nNo audio files found")
        return
    
    if not DEFAULT_COVER.is_file():
        print(f"\n[WARN] DEFAULT_COVER not found at {DEFAULT_COVER}")
        print("       Tracks without matching images will be skipped.\n")
    
    print(f"\nFound {len(audio_files)} audio files. Building track folders...")
    
    # Step 4: Process all audio files
    track_num = 1
    created_tracks = 0
    
    for audio_path in audio_files:
        result = process_audio_file(audio_path, track_num, youtube_downloader)
        if result:
            track_num += 1
            created_tracks += 1
    
    print(f"\n✓ Done. Created {created_tracks} track folders in '{INPUT_DIR}'.")


def interactive_mode():
    """Interactive mode for downloading YouTube URLs"""
    downloader = YouTubeDownloader()
    
    while True:
        print("\n" + "="*50)
        print("YouTube Audio Downloader")
        print("="*50)
        print("1. Download a SINGLE YouTube video")
        print("2. Download a YouTube PLAYLIST")
        print("3. Get video information")
        print("4. Process downloaded files into jukebox format")
        print("5. Exit")
        
        choice = input("\nEnter choice (1-5): ").strip()
        
        if choice == "1":
            url = input("Enter YouTube video URL: ").strip()
            if not url:
                continue
            
            print(f"Format: {AUDIO_FORMAT.upper()}")
            if AUDIO_FORMAT == 'mp3':
                print("Bitrate options: 128, 192, 256, 320 (kbps)")
                quality = input(f"Enter bitrate [{AUDIO_QUALITY}]: ").strip() or AUDIO_QUALITY
            else:
                print("Quality options: 0 (best) to 9 (worst)")
                quality = input(f"Enter quality [0]: ").strip() or "0"
            
            result = downloader.download_single_video(url, quality=quality)
            if result:
                print(f"\n✓ Successfully downloaded: {result.name}")
        
        elif choice == "2":
            url = input("Enter YouTube playlist URL: ").strip()
            if not url:
                continue
            
            max_items = None
            if input("Limit number of videos? (y/n): ").strip().lower() == 'y':
                try:
                    max_items = int(input("Maximum videos to download: "))
                except:
                    print("Invalid number, downloading all.")
            
            result = downloader.download_playlist(url, max_items=max_items)
            if result:
                print(f"\n✓ Downloaded {len(result)} videos")
        
        elif choice == "3":
            url = input("Enter YouTube URL: ").strip()
            info = downloader.get_video_info(url)
            if info:
                print(f"\nTitle: {info.get('title')}")
                print(f"Channel: {info.get('uploader')}")
                print(f"Duration: {info.get('duration', 0)} seconds")
                if 'entries' in info:
                    print(f"Playlist items: {len(info.get('entries', []))}")
        
        elif choice == "4":
            print("Processing files into jukebox format...")
            main()
            print("\nReturning to interactive mode...")
        
        elif choice == "5":
            print("Goodbye!")
            break
        
        else:
            print("Invalid choice")


if __name__ == "__main__":
    print("\nYouTube Audio Downloader with yt-dlp")
    print("-" * 40)
    print(f"Audio format: {AUDIO_FORMAT.upper()}")
    print(f"Download folder: {YOUTUBE_DOWNLOAD_DIR}")
    print("\nNote: Single video URLs with '&list=' in them might be detected as playlists.")
    print("Use the SINGLE video option for individual songs, even if URL contains playlist info.")
    
    if not YOUTUBE_URLS_FILE:
        run_interactive = input("\nRun in interactive mode? (y/n): ").strip().lower()
        if run_interactive == 'y':
            interactive_mode()
        else:
            main()
    else:
        main()