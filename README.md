# ROS Repository Template

Clone this repository for debian packaging of ROS nodes with [Continuous Integration and Deployment](https://automodality.atlassian.net/wiki/spaces/AUTOMOD/pages/668664251/Development+Lifecycle).


## Create a Repo from the Template

1. Create a [new Repository](https://github.com/organizations/AutoModality/repositories/new).
1. Name using `am_lower_case_with_underscores`. Notice the prefix with `am` to avoid package conflicts.
1. Choose `ros_debian_template` as the `Repository template`. 
1. Clone your new repository to your local development computer.
1. Run `repo-setup.sh`
1. Update the README to explain the purpose of this repo.
1. Commit your changes and push to github

```
git commit -a -m "New changes"
git push origin master
```

### Repository Settings

Github does not copy any settings from the template into your new repo.  As the creator of the repo, you'll have the ability to Administer the repo and can modify the settings to satisfy the requirements of our libraries.

1. Go to `Settings` in your new repo
1. Options -> Check `Automatically delete head branches` to remove branches upon PR merge
1. Manage Access -> `Invite Teams or People`
    1. Choose `AM Development` 
    1. Check `Maintain`
    1. `Add ...`
1. Branches -> `Add Rule` 
    1. `Branch Name Pattern` = `master`
    1. Check `Require status checks to pass before merging`
    1. Check `Require branches to be up to date before merging`
    1. Check `Package if released (self-hosted, linux, arm64)`
    1. Check `Package if released (ubuntu-18.04)`
    1. Check `build`
    1. Create
1. Secrets -> `Add New Secret`
    1. CLOUDSMITH_READ_DEV_ENTITLEMENT = See RPass `Cloudsmith Read Bot` Notes
    1. CLOUDSMITH_READ_RELEASE_ENTITLEMENT = See RPass `Cloudsmith Read Bot` Notes
    1. CLOUDSMITH_API_KEY = See RPass `Cloudsmith Push Bot for CI` Notes
1. Actions -> `Add Runner` -> `Linux` -> `ARM64`
    1. See [Wiki Instructions](https://automodality.atlassian.net/wiki/spaces/AUTOMOD/pages/758382915/Continuous+Integration+with+Github+Actions#TX2-ARM-Builder) or ask for help in [#config-mgmt](https://automodality.slack.com/archives/CRMR0V4P7)
# Building your first package

## Install the required tools for building packages
```
sudo apt-get install build-essential fakeroot devscripts
```

## Try building your package

```
debian/rules clean
fakeroot debian/rules binary
```



# Pull updates from template

We'll make changes to the template that you'll want to have in the future.  
The setup script adds the template as a remote repo, so just call:

```
git merge template/master --allow-unrelated-histories
```
