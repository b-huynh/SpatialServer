from threading import Thread
import subprocess
import os
import json

from .sfm_processing import SfmProcessor


potree_exec = '/home/jalexander/software/PotreeConverter/build/PotreeConverter/PotreeConverter'

def unzip_and_process(filename, image_set):
    working_dir = image_set.get_working_dir()
    if not os.path.exists(working_dir):
        os.makedirs(working_dir)
    img_dir = image_set.get_img_dir()

    set_data = json.loads(image_set.data)
    if image_set.status != 'uploaded':
        return

    args = [
        'unzip',
        '-j',
        '-d',
        img_dir,
        filename
    ]

    try:
        output = subprocess.check_output(args, stderr=subprocess.STDOUT)
        os.remove(filename)
    except subprocess.CalledProcessError as err:
        print('Error unzipping file:', err.returncode)
        print(bytes.decode(err.output))
        return

    image_set.status = 'unzipped'
    image_set.save()

    html_dir = image_set.get_html_dir()

    processor = SfmProcessor(sfm_source=img_dir, sfm_dest=working_dir, html_dest=html_dir,
                             potree_executable=potree_exec)
    print('Starting Colmap reconstruction')
    processor.run_colmap()
    print('Finished Colmap reconstruction')

    image_set.status = 'reconstructed'
    image_set.save()

    if not os.path.exists(html_dir):
        os.makedirs(html_dir)

    processor.convert_point_cloud()
    processor.convert_dense_ply_file()
    print('Finished model file conversion')
    processor.run_potree_convert()
    print('Finished HTML page generation with Potree')
    image_set.status = 'potree'
    image_set.save()


def start_thread_processing_on_zip(filename, image_set):
    thread = Thread(target=unzip_and_process, args=(filename, image_set))
    thread.start()
