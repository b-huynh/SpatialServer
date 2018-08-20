from django.db import models

import os

class ImageSet(models.Model):
    desc = models.CharField(max_length=200)
    ident = models.CharField(max_length=200, db_index=True)
    data = models.TextField(default='{}')
    status = models.CharField(max_length=100, default='')
    upload_date = models.DateTimeField()

    def get_working_dir(self):
        return os.path.join('sfm_files', self.ident)

    def get_dense_dir(self):
        return os.path.join(self.get_working_dir(), 'dense')

    def get_latest_dense_dir(self):
        subdir = sorted(os.listdir(self.get_dense_dir()))[-1]
        return os.path.join(self.get_dense_dir(), subdir)

    def get_pointcloud_file(self):
        return os.path.join(self.get_latest_dense_dir(), 'fused.ply')

    def get_img_dir(self):
        return os.path.join(self.get_working_dir(), 'img')

    def get_html_dir(self):
        return os.path.join('potree', self.ident)

    def get_view_html(self):
        return os.path.join(self.get_html_dir(), 'view.html')


class Alignment(models.Model):
    ident1 = models.CharField(max_length=200, db_index=True)
    ident2 = models.CharField(max_length=200, db_index=True)
    type = models.CharField(max_length=10, db_index=True, default='user')
    matrix = models.CharField(max_length=200)
    points1 = models.CharField(max_length=200)
    points2 = models.CharField(max_length=200)
    upload_date = models.DateTimeField()
