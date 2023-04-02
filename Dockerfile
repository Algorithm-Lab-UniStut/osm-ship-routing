FROM golang:1.19-alpine AS build

WORKDIR /app

# Copy go.mod and go.sum for installing dependencies
COPY go.mod .
COPY go.sum .

# Download dependencies
RUN go mod download

# Copy source code and additional resources
COPY . .

# Build server
RUN go build -o osm-server cmd/server/main.go

RUN adduser -D nonroot


FROM alpine:latest

WORKDIR /

COPY --from=build /etc/passwd /etc/passwd

COPY --from=build /app/osm-server /app/osm-server

USER nonroot

EXPOSE 8081

CMD ["/app/osm-server"]
