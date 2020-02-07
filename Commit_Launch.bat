git push origin --delete Launch
git branch -D Launch
git checkout -b Launch
git push -u origin Launch
git branch -D Launch
git fetch upstream
git checkout master
git reset --hard upstream/master
git push origin master -f