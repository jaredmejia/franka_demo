import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--path', type=str, default='/home/gaoyue/dev/franka_demo/logs/4-1-scooping-plate-1'\
)
parser.add_argument('--novideo', action='store_true')
args = parser.parse_args()


# Save robot video as avi using make_video
# Save microphone glove as wav using existing script
combine_cmd = 'ffmpeg -i ' + video_filename + '.avi -i ' + wav_filename + \
              '.wav -c:v copy -c:a aac -map 0:v:0 -map 1:a:0 -filter:a "volume=4.0" ' + \
              video_filename +'_gripperaudio.mp4 -hide_banner -loglevel error &'
subprocess.Popen(combine_cmd, shell=True,
                 stdout=subprocess.PIPE,
                 stderr=subprocess.PIPE)
print('Saved combined video file to ' + filename + str(delay)+'_gloveaudio.mp4'
      
