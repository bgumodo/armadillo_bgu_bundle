from __future__ import print_function

import requests
import json
from StringIO import StringIO

_url = 'http://csdlsrv1:8383'
_maxNumRetries = 10

def processRequest(data):
    params = dict()

    headers = dict()
    headers['Content-Type'] = 'application/octet-stream'

    json = None

    response = requests.request('post', _url, json=json, data=data, headers=headers, params=params)

    return response.content


def main():
    # Load raw image file into memory
    pathToFileInDisk = 'sample_image.jpg'
    with open(pathToFileInDisk, 'rb') as f:
        data = f.read()

    result = processRequest(data)

    if result is not None:
        string_io = StringIO(result)
        result_dict = json.load(string_io)
        print(result_dict)


if __name__ == '__main__':
    main()