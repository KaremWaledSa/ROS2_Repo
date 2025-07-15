#!/bin/bash

# Ask for your commit message
read -p "Enter your commit message: " msg

git add .
git commit -m "$msg"
git push origin main

