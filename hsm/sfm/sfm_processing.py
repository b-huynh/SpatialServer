import argparse
import subprocess
import os


class SfmProcessor:
    def __init__(self, sfm_source=None, sfm_dest=None, html_dest=None, potree_executable=None):
        self.sfm_source = sfm_source
        self.sfm_dest = sfm_dest
        self.html_dest = html_dest
        self.potree_executable = potree_executable
        self.converted_dir = os.path.join(sfm_dest, 'converted')
        self.model_dir = os.path.join(sfm_dest, 'sparse')
        self.ply_file_name = 'points.ply'
        self.las_file_name = 'points.las'

    def run_colmap(self, use_gpu=True):
        args = [
            'colmap',
            'automatic_reconstructor',
            '--image_path',
            self.sfm_source,
            '--workspace_path',
            self.sfm_dest,
            '--use_gpu'
        ]
        if use_gpu:
            args.append('1')
        else:
            args.append('0')

        try:
            output = subprocess.check_output(args, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as err:
            print('Error running colmap reconstructor:', err.returncode)
            print(bytes.decode(err.output))
            raise err

    def convert_point_cloud(self):
        if not os.path.exists(self.model_dir):
            mess = 'Error in colmap conversion. Model source directory does not exist.'
            print(mess)
            raise IOError(mess)

        subdir = sorted(os.listdir(self.model_dir))[-1]
        latest_model_dir = os.path.join(self.model_dir, subdir)

        if not os.path.exists(self.converted_dir):
            os.makedirs(self.converted_dir)

        ply_file = os.path.join(self.converted_dir, self.ply_file_name)
        args = [
            'colmap',
            'model_converter',
            '--input_path',
            latest_model_dir,
            '--output_path',
            ply_file,
            '--output_type',
            'PLY'
        ]

        try:
            output = subprocess.check_output(args, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as err:
            print('Error running colmap model converter:', err.returncode)
            print(bytes.decode(err.output))
            raise err

    def convert_ply_file(self):
        if not os.path.exists(self.converted_dir):
            os.makedirs(self.converted_dir)

        ply_file = os.path.join(self.converted_dir, self.ply_file_name)
        las_file = os.path.join(self.converted_dir, self.las_file_name)
        args = [
            'pdal',
            'translate',
            ply_file,
            las_file
        ]

        try:
            output = subprocess.check_output(args, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as err:
            print('Error running pdal model converter:', err.returncode)
            print(bytes.decode(err.output))
            raise err

    def run_potree_convert(self):
        if not os.path.exists(self.html_dest):
            mess = 'Error in potree conversion. HTML destination directory does not exist.'
            print(mess)
            raise IOError(mess)

        las_file = os.path.join(self.converted_dir, self.las_file_name)
        args = [
            self.potree_executable,
            las_file,
            '-o',
            self.html_dest,
            '--generate-page',
            'view'
        ]
        print(args)

        try:
            output = subprocess.check_output(args, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as err:
            print('Error running potree converter:', err.returncode)
            print(bytes.decode(err.output))
            raise err



def get_parser():
    parser = argparse.ArgumentParser(
        description='Python script for running SFM jobs.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--full_pipeline', action='store_true',
                        help='Run the full Colmap and Potree pipeline')
    parser.add_argument('--run_colmap', action='store_true',
                        help='Run Colmap SFM processing on a set of images')
    parser.add_argument('--convert_col_las', action='store_true',
                        help='Convert Colmap model output to PLY format and PLY file to LAS format')
    parser.add_argument('--convert_ply_las', action='store_true',
                        help='Convert PLY file to LAS format')
    parser.add_argument('--run_potree', action='store_true',
                        help='Convert LAS model to HTML page using Potree')

    parser.add_argument('--source_dir', type=str,
                        help='Directory containing images to process')
    parser.add_argument('--out_dir', type=str,
                        help='Directory to output SFM results (usually the parent of source_dir)')
    parser.add_argument('--html_dir', type=str,
                        help='Directory to output HTML page (note: should be a different directory for each HTML page)')
    parser.add_argument('--potree_exec', type=str,
                        help='The PotreeConverter executable')
    return parser


if __name__ == '__main__':
    args = get_parser().parse_args()
    if args.full_pipeline:
        if args.source_dir is not None and args.out_dir is not None:
            processor = SfmProcessor(sfm_source=args.source_dir, sfm_dest=args.out_dir, html_dest=args.html_dir, potree_executable=args.potree_exec)
            processor.run_colmap()
            print('Finished Colmap reconstruction')
            processor.convert_point_cloud()
            processor.convert_ply_file()
            print('Finished model file conversion')
            processor.run_potree_convert()
            print('Finished HTML page generation with Potree')
        else:
            print('Please provide source, model destination, and HTML destination directories as well as the location of the PotreeConverter executable.')
    else:
        if args.run_colmap:
            if args.source_dir is not None and args.out_dir is not None:
                processor = SfmProcessor(sfm_source=args.source_dir, sfm_dest=args.out_dir)
                processor.run_colmap()
                print('Finished Colmap reconstruction')
            else:
                print('Please provide source and model destination directories.')
        if args.convert_col_las:
            if args.out_dir is not None:
                processor = SfmProcessor(sfm_dest=args.out_dir)
                processor.convert_point_cloud()
                processor.convert_ply_file()
                print('Finished model file conversion')
            else:
                print('Please provide model destination directory.')
        if args.convert_ply_las:
            if args.out_dir is not None:
                processor = SfmProcessor(sfm_dest=args.out_dir)
                processor.convert_ply_file()
                print('Finished model file conversion')
            else:
                print('Please provide model destination directory.')
        if args.run_potree:
            if args.out_dir is not None and args.html_dir is not None and args.potree_exec is not None:
                processor = SfmProcessor(sfm_dest=args.out_dir, html_dest=args.html_dir, potree_executable=args.potree_exec)
                processor.run_potree_convert()
                print('Finished HTML page generation with Potree')
            else:
                print('Please provide model destination and HTML destination directories as well as the location of the PotreeConverter executable.')

