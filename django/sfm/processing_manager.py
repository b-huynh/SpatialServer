from threading import Thread
import subprocess
import os
import json

from .sfm_processing import SfmProcessor
from upload.models import ImageSet


potree_exec = '/home/jalexander/software/PotreeConverter/build/PotreeConverter/PotreeConverter'

def unzip_and_process(filename, image_set):
    working_dir = image_set.get_working_dir()
    if not os.path.exists(working_dir):
        os.makedirs(working_dir)
    img_dir = image_set.get_img_dir()

    if image_set.status != 'uploaded':
        return

    unzip_image_set(filename, image_set)

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

    processor.convert_dense_ply_file()
    print('Finished model file conversion')
    processor.run_potree_convert()
    print('Finished HTML page generation with Potree')
    image_set.status = 'potree'
    image_set.save()


def check_status_and_process(ident):
    image_set = ImageSet.objects.get(ident=ident)
    if image_set.status == 'uploaded':
        print('Cannot handle unzipping image sets')

    processor = SfmProcessor(sfm_source=image_set.get_img_dir(), sfm_dest=image_set.get_working_dir(),
                             html_dest=image_set.get_html_dir(), potree_executable=potree_exec)

    if image_set.status == 'unzipped':
        point_file = image_set.get_pointcloud_file()
        if os.path.isfile(point_file):
            image_set.status = 'reconstructed'
            image_set.save()
        else:
            print('Starting Colmap reconstruction')
            processor.run_colmap()
            print('Finished Colmap reconstruction')

            image_set.status = 'reconstructed'
            image_set.save()

    if image_set.status == 'reconstructed':
        html_file = image_set.get_view_html()
        if os.path.isfile(html_file):
            image_set.status = 'potree'
            image_set.save()
        else:
            html_dir = image_set.get_html_dir()
            if not os.path.exists(html_dir):
                os.makedirs(html_dir)

            processor.convert_dense_ply_file()
            print('Finished model file conversion')
            processor.run_potree_convert()
            print('Finished HTML page generation with Potree')
            image_set.status = 'potree'
            image_set.save()



def unzip_image_set(filename, image_set):
    args = [
        'unzip',
        '-j',
        '-d',
        image_set.get_img_dir(),
        filename
    ]

    try:
        output = subprocess.check_output(args, stderr=subprocess.STDOUT)
        os.remove(filename)
    except subprocess.CalledProcessError as err:
        print('Error unzipping file:', err.returncode)
        print(bytes.decode(err.output))
        return


def start_thread_processing_on_zip(filename, image_set):
    thread = Thread(target=unzip_and_process, args=(filename, image_set))
    thread.start()
