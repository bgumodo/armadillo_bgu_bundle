from __future__ import print_function

import os
import requests
import json
from StringIO import StringIO

_url = os.getenv('VISION_SERVER', '')
_maxNumRetries = 10


def parse_query(query):
    params = {'query': query}

    headers = dict()
    headers['Content-Type'] = 'application/octet-stream'

    response = requests.request('get', _url + '/parse_query', headers=headers, params=params)

    string_io = StringIO(response.content)
    json_result = json.load(string_io)

    return json_result


def processRequest(data):
    params = dict()

    headers = dict()
    headers['Content-Type'] = 'application/octet-stream'

    response = requests.request('post', _url + '/maskrcnn', data=data, headers=headers, params=params)

    string_io = StringIO(response.content)
    json_result = json.load(string_io)

    classes_dict = {}
    for object in json_result['class_names']:
        if object not in classes_dict:
            classes_dict[object] = 1
        else:
            classes_dict[object] += 1

    objects_string = ''
    for i, object in enumerate(classes_dict):
        if classes_dict[object] == 1:
            objects_string += str(classes_dict[object]) + ' ' + object.split('|')[0]
        else:
            objects_string += str(classes_dict[object]) + ' ' + object.split('|')[1]

        if i < len(classes_dict) - 2:
            objects_string += ', '
        elif i == len(classes_dict) - 2:
            objects_string += ' and '

    result = {'result': json_result, 'objects_string': objects_string}

    return result


def main():
    # Load raw image file into memory

    pathToFileInDisk = 'sample_image.jpg'
    with open(pathToFileInDisk, 'rb') as f:
        data = f.read()

    result = processRequest(data)

    print(result)

    print(parse_query("Show me the cup"))


if __name__ == '__main__':
    main()