## Cross-Session Communication Protocol

You are one of several Codex agents working on Piccard.

When you need to communicate with another Codex session:

1. Use `list_threads` to identify the correct destination session.
2. Use `send_message_to_thread` to communicate directly with that session. Do not ask the user to manually relay messages.
3. Include a unique message ID, your session name, the relevant project, the requested action, and whether a response is required.
4. If you need information from another session, use `read_thread` to inspect its latest state when permitted.
5. If a message is important, verify that it reached the intended thread. Request acknowledgment from the receiving agent.
6. Never assume a successful tool call means the receiving agent has processed the message.
7. Avoid duplicate sends. If the tool returns an error, inspect the destination thread before retrying.
8. If native messaging is unavailable, report the limitation rather than claiming delivery.

When receiving a message from another agent, acknowledge it and incorporate the information into your work. If the message conflicts with ongoing work, explain the conflict to the sender.

Use native cross-thread messaging rather than `codex queue` for inter-agent communication whenever the native tool is available.
