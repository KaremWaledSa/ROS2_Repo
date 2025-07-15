#!/bin/bash
git add .
git commit -m "update"
git push
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519

