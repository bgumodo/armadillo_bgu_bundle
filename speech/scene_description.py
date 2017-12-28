from __future__ import print_function

import os
import time
import requests

_url = 'https://westeurope.api.cognitive.microsoft.com/vision/v1.0/analyze'
_key = os.getenv('MICROSOFT_VISION_KEY', '')
_maxNumRetries = 10

def processRequest(data):
    """
    Helper function to process the request to Project Oxford

    Parameters:
    json: Used when processing images from its URL. See API Documentation
    data: Used when processing image read from disk. See API Documentation
    headers: Used to pass the key information and the data type request
    """

    # Computer Vision parameters
    params = {'visualFeatures': 'Description'}

    headers = dict()
    headers['Ocp-Apim-Subscription-Key'] = _key
    headers['Content-Type'] = 'application/octet-stream'

    json = None

    retries = 0
    result = None

    while True:

        response = requests.request('post', _url, data=data, headers=headers, params=params)

        if response.status_code == 429:

            print("Message: %s" % (response.json()['error']['message']))

            if retries <= _maxNumRetries:
                time.sleep(1)
                retries += 1
                continue
            else:
                print('Error: failed after retrying!')
                break

        elif response.status_code == 200 or response.status_code == 201:

            if 'content-length' in response.headers and int(response.headers['content-length']) == 0:
                result = None
            elif 'content-type' in response.headers and isinstance(response.headers['content-type'], str):
                if 'application/json' in response.headers['content-type'].lower():
                    result = response.json() if response.content else None
                elif 'image' in response.headers['content-type'].lower():
                    result = response.content
        else:
            print("Error code: %d" % (response.status_code))
            print("Message: %s" % (response.json()['message']))


        break

    return result


# Load raw image file into memory
# pathToFileInDisk = 'sample_image.jpg'
# with open(pathToFileInDisk, 'rb') as f:
#     data = f.read()
#
# result = processRequest(data)
#
# if result is not None:
#     print(result['description']['captions'][0]['text'])