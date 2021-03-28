# Flow

C++14, header-only library for multi-stream data synchronization.

## What is this used for?

This library is meant for generating groups of data from separate series. The core problems it is meant to address are:

- How do we know which elements of data relate to one other across multiple series?
- How do we know when this data is ready to be retrieved ("captured") for further use?
- How do we capture different types of data uniformly and with minimal overhead?

In addressing these problems, this library enables data-driven event execution using data collected from distinct streaming series.
