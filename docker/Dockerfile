FROM python:3.10-slim

WORKDIR /app
COPY pyproject.toml /app/pyproject.toml
COPY ccoli /app/ccoli
COPY server /app/server
RUN pip install --no-cache-dir -e '/app[runtime]'
WORKDIR /app/server

CMD ["python", "server.py"]
