#pragma once
#include <coroutine>
#include <exception> // terminate
#include <utility> // move

template <typename T>
struct generator {
	union promise_type {
		promise_type() {}
		generator get_return_object() { return generator{this}; };
		std::suspend_always yield_value(T value) { new(&current_value_) T{std::move(value)}; return {}; }
		std::suspend_always initial_suspend() { return {}; }
		std::suspend_always final_suspend() noexcept { return {}; }
		void return_void() {}
		void unhandled_exception() { std::terminate(); }

		T const &get_value() const { return current_value_; }

	private:
		T current_value_;
	};

	struct iterator {
		iterator(std::coroutine_handle<promise_type> coro, bool done) :
			coro_{coro},
			done_{done}
		{
		}

		iterator& operator++() {
			coro_.resume();
			done_ = coro_.done();
			return *this;
		}

		bool operator== (iterator const &rhs) const { return done_ == rhs.done_; }
		bool operator!= (iterator const &rhs) const { return !(*this == rhs); }
		T const &operator* () const { return coro_.promise().get_value(); }
		T const *operator-> () const { return &(operator*()); }

	private:
		std::coroutine_handle<promise_type> coro_;
		bool done_;
	};

	iterator begin() const { promise_.resume(); return {promise_, promise_.done()}; }
	iterator end() const { return {promise_, true}; }

	generator(generator const&) = delete;
	generator(generator &&rhs) : promise_{rhs.promise_} { rhs.promise_ = nullptr; }
	~generator() { if (promise_) promise_.destroy(); }

private:
	std::coroutine_handle<promise_type> promise_;

	explicit generator(promise_type *promise) :
		promise_{std::coroutine_handle<promise_type>::from_promise(*promise)}
	{}
};
