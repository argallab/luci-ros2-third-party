# Third Party Node

## Summary 
This repo is for open source packages provided by researchers and the community using the LUCI ROS2 SDK

## How to contribute
This project welcomes third-party code via pull requests.

You are welcome to propose and discuss enhancements using project issues.

Branching Policy: The latest tag on the main branch is considered stable. The main branch is the one where all contributions must be merged before being promoted to a tagged release. If you plan to propose a patch, please commit into its own feature branch.

All contributions must comply with the project's standards:

Every example / source file must refer to LICENSE
Every example / source file must include correct copyright notice

Depending on your language used it is expected you follow one of two styling guides
- Python use [PEP8](https://peps.python.org/pep-0008/) Standard
- C++ use [clang](https://clang.llvm.org/docs/ClangFormat.html) formatting 

Please familiarize yourself with the Apache License 2.0 before contributing.

## Contributing to existing LUCI Packages
If there are bugs or feature you would like to see addressed / added to existing LUCI ROS2 packages, please setup an issue in the [luci-ros2-sdk repo](https://github.com/lucimobility/luci-ros2-sdk). As the individual package's code is kept private LUCI will use the public SDK issues tracker as a method of improving existing packages in future releases.

## Example and general use based changes
For changes that are in reference to examples or general information about how to use the SDK the [luci-ros2-sdk repo](https://github.com/lucimobility/luci-ros2-sdk) is where PRs should be made

## Added functionality or new package changes
For changes that are an additional feature / package or is a fix to an existing third-party package please open a PR and issue in the this repo [third-party repo](https://github.com/lucimobility/luci-ros2-third-party). This repo was setup to allow a single point of collaboration for all users of the SDK if they desire to push the project forward with their contributions. 

We see this SDK as a method to get researchers and hobbyists alike all working towards the goal of a better world for those who use a LUCI product. 

It should also be noted that based on the usefulness of a contributed package or its direct reliance on LUCI we may decide to pull individual packages or code out of the third party repo and into its own separate repo / LUCI package binary. 

An example of this would be if a LUCI specific URDF were developed we would pull this into a specific URDF package separate from third party.

## How to make a new feature PR
In order to get a new feature or change into the next official LUCI ROS2 SDK release please follow the steps listed below

- Fork the Project
- Create your Feature Branch (git checkout -b feature/AmazingFeature)
- Commit your Changes (git commit -m 'Add some AmazingFeature')
- Push to the Branch (git push origin feature/AmazingFeature)
- Open a Pull Request

## Guidelines for Pull Requests
How to get your contributions merged smoothly and quickly.

- Create small PRs that are narrowly focused on addressing a single concern. Create multiple PRs to address different concerns.

- For speculative changes, consider opening an issue and discussing it first.

- Provide a good PR description as a record of what change is being made and why it was made. Link to a GitHub issue if it exists.

- If you are adding a new file, make sure it has the copyright message template at the top as a comment. You can copy over the message from an existing file and update the year.

- Unless your PR is trivial, you should expect there will be reviewer comments that you'll need to address before merging. We expect you to be reasonably responsive to those comments, otherwise the PR will be closed after 2-3 weeks of inactivity.

- Maintain clean commit history and use meaningful commit messages. PRs with messy commit history are difficult to review and won't be merged. Use rebase -i to curate your commit history.

- Keep your PR up to date with upstream/main (if there are merge conflicts, we can't really merge your change).

- Depending on the aggressiveness of a change reviewers may ask that you include unit tests to make sure there is a proper level of code coverage.

- Exceptions to the rules can be made if there's a compelling reason for doing so.


## Building 
In order to build and develop this package users can use the LUCI provided development docker image that comes pre installed with ROS2 and all supporting dev tools. To get this image run `docker run -d -it -p 8765:8765 luci.jfrog.io/ros2-sdk-docker-local/luci-sdk-development-image:latest`. This will get the lates development image provided from the LUCI artifactory. This image is updated at minimum every major release of the SDK. 

Once running the development image run the following steps inside of the container to build the third_party package.

1. `git clone git@github.com:lucimobility/luci-ros2-third-party.git`
2. `cd luci-ros2-third-party`
3. `colcon build`
4. `source install/setup.bash`

At this point any node should be available to your system through a `ros2 run` command.

<b>NOTE: Theres is also a provided `build-package.sh` script that can be used to build the ROS package and an installable `.deb` file. This is what the LUCI team uses for its automatic deployments.</b>

<b>Please be aware that the `deploy-package.sh` and the `sign-package.sh` scripts are NOT to be ran by anyone outside of the LUCI company. They will fail if ran by anyone without proper credentials and are only for the automated release process used by LUCI. </b>


## Releasing new version (FOR LUCI EMPLOYEES ONLY) ##
When a new version of this package is ready to be released there are a couple steps to follow. It is important to note that most of the process is automated for convenience and the process should be just a couple of button clicks. 

### Steps ### 
1. Update release version
    - This should be its own separate PR and should only update the package.xml `<version> </version>` tag. 
    - LUCI follows [semver](https://semver.org/) style versioning so MAJOR.MINOR.PATCH versions are expected.
    - It is okay to not put out versions until multiple changes have happened to the code. 
2. Once the version increment is merged you simply need to create an official release in github. Make sure you make the release version the same as what is now in `package.xml`. We have chosen to keep github release and package version in sync.
    - This should trigger an action to auto run called `Create and Sign Package` which you can monitor in the github actions panel. This should grab the released code, build it, make an installable .deb file, gdb sign it and push it to jrog artifactory.

If everything went smoothly congratulations the new package will be released and publicly distributable. 


<b>NOTE: Once a PR is merged into the `main` branch the docs site in the `next` version will update with it that evening.</b>
