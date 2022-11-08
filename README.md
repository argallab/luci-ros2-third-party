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

Depending on your language used it is expected you follow one of two syling guides
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

