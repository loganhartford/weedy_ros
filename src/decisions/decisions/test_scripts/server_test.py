import requests

# Try different addresses based on how Docker is configured
URLS = [
    "http://localhost:8000",                # Works if using --network host
    # "http://host.docker.internal:8080",     # Works on Mac/Windows
    # "http:/0.0.0.0:8080",              # Replace with the Pi's actual IP
    # "http://weedy:8000"
]

for url in URLS:
    try:
        print(f"Trying {url}...")
        response = requests.get(url, timeout=3)
        print(f"Response from {url}: {response.text}")
        break  # Stop after first successful response
    except requests.exceptions.RequestException as e:
        print(f"Failed to connect to {url}: {e}")
