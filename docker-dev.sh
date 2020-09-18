set -e

# start docker dev container
# docker system prune -f
docker run -it -v .:/home/mobiledevops/app mobiledevops/android-sdk-image bash

# inside the dev container
# - ./gradlew clean assembleRelease
# - adb connect <host:port>
# - adb install -r TeamCode/build/outputs/apk/release/TeamCode-release.apk

# set up nginx proxy for ftc dashboard
#
# add custom location
# ws:// -> wss://
# :8000 -> /wss
# :8081 -> /wss8081
#
# rewrite web socket url
# location /dash {
#   proxy_pass http://10.0.0.169:8080;
#   proxy_set_header Accept-Encoding "";
#   sub_filter_types application/javascript text/javascript;
#   sub_filter 'ws://' 'wss://';
#   sub_filter '{window.location.hostname}:8000' '{window.location.hostname}/wss';
#   sub_filter_once off;
# }
# location /js {
#   proxy_pass http://10.0.0.169:8080;
#   proxy_set_header Accept-Encoding "";
#   sub_filter_types application/javascript text/javascript;
#   sub_filter 'ws://' 'wss://';
#   sub_filter "window.location.hostname + ':' + (parseInt(window.location.port) + 1)" "window.location.hostname + '/wss8081'";
#   sub_filter_once off;
# }
