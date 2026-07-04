import requests

url = "http://localhost:1234/v1/chat/completions"

data = {
    "model": "qwen/qwen3.5-9b",
    "messages": [
        {"role": "user", "content": "Napisz funkcję w Pythonie, która sortuje listę"}
    ],
    "stream": False
}

r = requests.post(url, json=data)
print(r.json()["choices"][0]["message"]["content"])