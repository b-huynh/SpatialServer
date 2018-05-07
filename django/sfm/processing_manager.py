from threading import Thread
import subprocess
import os

from .sfm_processing import SfmProcessor


potree_exec = '/home/jalexander/software/PotreeConverter/build/PotreeConverter/PotreeConverter'

def unzip_and_process(filename, ident):
    working_dir = os.path.join('sfm_files', ident)
    if not os.path.exists(working_dir):
        os.makedirs(working_dir)
    img_dir = os.path.join(working_dir, 'img')

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

    html_dir = os.path.join('potree', ident)

    processor = SfmProcessor(sfm_source=img_dir, sfm_dest=working_dir, html_dest=html_dir,
                             potree_executable=potree_exec)
    print('Starting Colmap reconstruction')
    processor.run_colmap()
    print('Finished Colmap reconstruction')

    if not os.path.exists(html_dir):
        os.makedirs(html_dir)

    processor.convert_point_cloud()
    processor.convert_dense_ply_file()
    print('Finished model file conversion')
    processor.run_potree_convert()
    print('Finished HTML page generation with Potree')


def start_thread_processing_on_zip(filename, ident):
    thread = Thread(target=unzip_and_process, args=(filename, ident))
    thread.start()
