#!/bin/bash
CERTIFICATES_DIRECTORY=../.certificates

if [ -d "$CERTIFICATES_DIRECTORY" ]; then
echo "Certificates are already generated, get out of there or remove them manually"
else
mkdir -p $CERTIFICATES_DIRECTORY
openssl genrsa -out $CERTIFICATES_DIRECTORY/key.pem 1024
openssl req -new -key $CERTIFICATES_DIRECTORY/key.pem -out $CERTIFICATES_DIRECTORY/request.pem
openssl x509 -req -days 9999 -in $CERTIFICATES_DIRECTORY/request.pem -signkey $CERTIFICATES_DIRECTORY/key.pem -out $CERTIFICATES_DIRECTORY/certificate.pem
openssl pkcs8 -topk8 -outform DER -in $CERTIFICATES_DIRECTORY/key.pem -inform PEM -out $CERTIFICATES_DIRECTORY/key.pk8 -nocrypt
fi
