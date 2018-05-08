from django.db import models

import os

class ImageSet(models.Model):
    desc = models.CharField(max_length=200)
    ident = models.CharField(max_length=200)
    data = models.TextField(default='{}')
    status = models.CharField(max_length=100, default='')
    upload_date = models.DateTimeField()

    def get_working_dir(self):
        return os.path.join('sfm_files', self.ident)

    def get_img_dir(self):
        return os.path.join(self.get_working_dir(), 'img')

    def get_html_dir(self):
        return os.path.join('potree', self.ident)
