# Google Calendar integration

ccoli uses Google Calendar OAuth, not a simple API key. You need:

- `GOOGLE_CLIENT_ID`
- `GOOGLE_CLIENT_SECRET`
- `GOOGLE_REFRESH_TOKEN`
- Optional: `GOOGLE_CALENDAR_ID`, defaults to `primary`
- Optional: `GOOGLE_CALENDAR_TIME_ZONE`, defaults to `Asia/Seoul`

## 1. Enable the Calendar API

1. Open [Google Cloud Console API Library](https://console.cloud.google.com/apis/library/calendar-json.googleapis.com).
2. Select or create a project.
3. Enable **Google Calendar API**.

Google's general guide for enabling Workspace APIs is here:
[Enable Google Workspace APIs](https://developers.google.com/workspace/guides/enable-apis).

## 2. Create OAuth credentials

1. Open [Google Auth Platform Clients](https://console.cloud.google.com/auth/clients).
2. Configure the OAuth consent screen if Google asks for it.
3. Add your own Google account as a test user while the app is in testing mode.
4. Create an OAuth client.

For the easiest refresh-token setup with OAuth Playground:

- Application type: `Web application`
- Authorized redirect URI: `https://developers.google.com/oauthplayground`

Google's OAuth client management guide is here:
[Manage OAuth clients](https://support.google.com/cloud/answer/6158849).

## 3. Get a refresh token

1. Open [OAuth 2.0 Playground](https://developers.google.com/oauthplayground).
2. Click the gear icon.
3. Enable **Use your own OAuth credentials**.
4. Paste the OAuth client ID and client secret.
5. In Step 1, enter this scope:

```text
https://www.googleapis.com/auth/calendar
```

6. Click **Authorize APIs** and approve access with your Google account.
7. Click **Exchange authorization code for tokens**.
8. Copy the `refresh_token`.

The full calendar scope is needed because ccoli supports list/create/update/delete event calls. For read-only use, the narrower scope is `https://www.googleapis.com/auth/calendar.readonly`, but create/update/delete will fail.

## 4. Configure ccoli

Run this from the repository root:

```bash
ccoli config integration set calendar-google \
  --client-id <GOOGLE_CLIENT_ID> \
  --client-secret <GOOGLE_CLIENT_SECRET> \
  --refresh-token <GOOGLE_REFRESH_TOKEN> \
  --calendar-id primary \
  --time-zone Asia/Seoul
```

Then verify local configuration:

```bash
ccoli config integration test calendar-google
```

Restart the server after changing credentials:

```bash
ccoli start
```

## Notes

- Secrets are written to `server/.env`, which is ignored by git.
- Do not put OAuth secrets in `server/config.yaml`.
- Google can revoke or expire refresh tokens. If Calendar starts returning auth errors, generate a new refresh token and rerun the `ccoli config integration set calendar-google ...` command.
- Event API reference: [Events: list](https://developers.google.com/workspace/calendar/api/v3/reference/events/list), [Events resource](https://developers.google.com/workspace/calendar/api/v3/reference/events).
