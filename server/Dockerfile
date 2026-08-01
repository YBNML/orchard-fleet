FROM python:3.12-slim
WORKDIR /app
COPY pyproject.toml ./
COPY fleet_server ./fleet_server
COPY web ./web
RUN pip install --no-cache-dir .
EXPOSE 8000
CMD ["uvicorn", "fleet_server.app:create_app", "--factory", "--host", "0.0.0.0", "--port", "8000"]
