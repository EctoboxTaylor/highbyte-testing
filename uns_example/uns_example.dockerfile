# Start from the Python 3.8 slim buster image
FROM python:3.12

# Install cron
RUN apt-get update && apt-get install -y \
    cron \
    vim

# Create and change to app directory
WORKDIR /app

# ********************************************************************************************************
# Python dependencies
# ********************************************************************************************************
# Copy the requirements file into the container
COPY requirements.txt .
# Activate the virtual environment and install the Python dependencies
RUN /bin/bash -c "pip install -r requirements.txt"

# Copy your Python script into the container
COPY . /app

# Copy your Python script into the container
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

RUN echo "ALL Done"
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]