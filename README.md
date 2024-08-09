
# Configuration

Il faut mettre la configuration suivante dans le fichier suivant qui est utilisé par le conteneur nginx:

fichier: /srv/docker/nginx/etc/nginx.conf

Mettre la configuration suivante à la fin de l'élément http {}:

        server {
            listen 443 ssl;
            server_name nodemcu.mikesbrewshop.com;

            ssl_certificate /etc/nginx/certificate/fullchain1.pem;
            ssl_certificate_key /etc/nginx/certificate/privkey1.pem;

            location / {
                proxy_pass http://10.0.0.143;
                proxy_http_version 1.1;
                proxy_set_header Upgrade $http_upgrade;
                proxy_set_header Connection "upgrade";
                proxy_set_header Host $host;
            }

            location /ws {
                proxy_pass http://10.0.0.143:3333;  # WebSocket server port
                proxy_http_version 1.1;
                proxy_set_header Upgrade $http_upgrade;
                proxy_set_header Connection "upgrade";
                proxy_set_header Host $host;
            }
        }

# TODO

- Permettre de faire un scan des DS18B20 et les associer aux fermenteurs, via la page web.
- Permettre de changer les paramètres PID de chaque fermenteur, via la page web.
- ...