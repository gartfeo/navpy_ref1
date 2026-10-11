import os
import zipfile

import requests


class SessionWithHeaderRedirection(requests.Session):
    AUTH_HOST = 'urs.earthdata.nasa.gov'

    def __init__(self, username, password):

        super().__init__()

        self.auth = (username, password)

    # Overrides from the library to keep headers when redirected to or from

    # the NASA auth host.

    def rebuild_auth(self, prepared_request, response):
        headers = prepared_request.headers
        url = prepared_request.url

        if 'Authorization' in headers:
            original_parsed = requests.utils.urlparse(response.request.url)
            redirect_parsed = requests.utils.urlparse(url)

            if (original_parsed.hostname != redirect_parsed.hostname) and \
                    redirect_parsed.hostname != self.AUTH_HOST and \
                    original_parsed.hostname != self.AUTH_HOST:
                del headers['Authorization']
        return


class SRTMDownloader:
    BASE_URL = "https://e4ftl01.cr.usgs.gov/MEASURES/SRTMGL1.003/2000.02.11/"
    BASE_URL_SUFFIX = ".SRTMGL1.hgt.zip"

    def __init__(self, username, password):
        self.session = SessionWithHeaderRedirection(username, password)

    def download_from_url(self, url, srtm_dir):
        filename = url[url.rfind('/') + 1:]

        # Create the directory if it doesn't exist
        if not os.path.exists(srtm_dir):
            os.makedirs(srtm_dir)

        # Create full path for the downloaded file
        file_path = os.path.join(srtm_dir, filename)

        try:
            response = self.session.get(url, stream=True)
            response.raise_for_status()

            with open(file_path, 'wb') as fd:
                for chunk in response.iter_content(chunk_size=1024 * 1024):
                    fd.write(chunk)
            print(f"Successfully downloaded {filename}")

            # Unzip the file
            with zipfile.ZipFile(file_path, 'r') as zip_ref:
                zip_ref.extractall(srtm_dir)  # extracts content to srtm_dir

            # Remove the zip file
            os.remove(file_path)
        except requests.exceptions.HTTPError as e:
            print(e)

    def download_by_coordinates(self, tile_name, dem_dir):
        tile_name_without_extension = os.path.splitext(tile_name)[0]  # This removes the extension
        hgt_url = f"{self.BASE_URL}{tile_name_without_extension}{self.BASE_URL_SUFFIX}"
        return self.download_from_url(hgt_url, dem_dir)
