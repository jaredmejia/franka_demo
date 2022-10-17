import argparse
import numpy as np
import os
import pandas as pd
import warnings

from glob import glob

parser = argparse.ArgumentParser(description="Aligning audio and images from teleop demonstrations")
parser.add_argument('--parent_dir', help='directory with <date>.txt and <date>/<img>.jpeg')

def get_sec(time_str):
    """Get Seconds from time."""
    hour, minute, second, second_decimal = time_str.split('.')
    return int(hour) * 3600 + int(minute) * 60 + int(second) + float('0.' + second_decimal)

def read_txt(file_path):
    with open(file_path, "r") as filehandle:
        data = []
        for line in filehandle:
            line = line[:-1]
            line = line.split(" ")
            if len(line) > 1:
                data.append(line)
            else:
                break
    time = [get_sec(line[0]) for line in data]
    time = np.asarray(time) - time[0] # Start time axis at 0s
    data = [line[1:] for line in data]
    data = np.array(data).astype(float)

    return time, data


def get_image_df(image_dir):
    image_list = glob(os.path.join(image_dir, '*color.jpeg'))
    image_df = pd.DataFrame(image_list, columns=['fname'])
    image_df['time'] = image_df['fname'].apply(lambda f: f.split('-')[-2])
    image_df = image_df.sort_values('time', axis=0).reset_index(drop=True)

    image_time_arr = image_df['time'].to_numpy().astype(float)
    image_df['abs_time_sec'] = (image_time_arr - image_time_arr[0]) / 1000.0

    return image_df


def audio_clip_indices(img_start, audio_time, img_dur=1/30):
    return np.where((img_start <= audio_time) & (audio_time <= img_start + img_dur))

def write_aligned_audio(audio_file):
    audio_time, audio_data = read_txt(audio_file)
    image_dir = audio_file[:-4]
    image_df = get_image_df(image_dir)

    num_images = len(glob(os.path.join(image_dir, '*color.jpeg')))
    print(f'num images in {image_dir}: {num_images}')

    num_bad_audio = 0
    for row in image_df.itertuples():
        audio_slice = audio_data[audio_clip_indices(row.abs_time_sec, audio_time)]
        if len(audio_slice) == 0:
            warnings.warn(f"Audio associated with {row.fname} is empty. No audio txt file saved")
            num_bad_audio += 1
        else:
            txt_path = f'{row.fname[:-4]}txt'
            np.savetxt(txt_path, audio_slice)
    
    num_txt = len(glob(os.path.join(image_dir, '*color.txt')))
    print(f'num txt files in {image_dir}: {num_txt}')
    print(f'num missing audio: {num_bad_audio}')
    return num_bad_audio


def main():
    args = parser.parse_args()

    txt_audio_files = glob(os.path.join(args.parent_dir, '*.txt'))

    total_bad_audio = 0
    for audio_file in txt_audio_files:
        total_bad_audio += write_aligned_audio(audio_file)

    print(f'Total missing audio: {total_bad_audio}')

if __name__ == "__main__":
    main()

