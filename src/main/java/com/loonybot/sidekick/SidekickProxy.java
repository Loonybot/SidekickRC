/// Common interface for all objects wrapped by the Sidekick proxy builder.
///
/// Copyright Andrew Goossen.
package com.loonybot.sidekick;

/// All Sidekick proxies implement this interface.
public interface SidekickProxy {
    Object unwrap(); // Return the original delegate of a wrapped proxy object
}
