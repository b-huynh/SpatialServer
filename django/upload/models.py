from django.db import models

class ImageSet(models.Model):
    desc = models.CharField(max_length=200)
    ident = models.CharField(max_length=200)
    data = models.TextField(default='{}')
    upload_date = models.DateTimeField()
